//
//  Publication.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-04.
//

import Foundation
import Atomics
import StdMsgs
import rpcobject
import Synchronization

internal struct PeerConnDisconnCallback: CallbackInterface, Sendable {
    
    let callback: SubscriberStatusCallback
    let subLink: SubscriberLink!
    let useTrackedObject: Bool
    weak var trackedObject: TrackableObject?
    
    init(callback: @escaping SubscriberStatusCallback,
         subLink: SubscriberLink,
         useTrackedObject: Bool = false,
         trackedObject: TrackableObject? = nil) {
        
        self.callback = callback
        self.subLink = subLink
        self.useTrackedObject = useTrackedObject
        self.trackedObject = trackedObject
    }
    
    @discardableResult
    func call() -> CallResult {
        if useTrackedObject {
            if trackedObject == nil {
                return .invalid
            }
        }
        
        let pub = SingleSubscriberPublisher(link: subLink)
        callback(pub)
        
        return .success
    }
    
    let ready = true
}

final class Publication: Sendable {
    let name: String
    let datatype: String
    let md5sum: String
    let messageDefinition: String
    let sequenceNr = ManagedAtomic<UInt32>(0)
    let pubCallbacks = SynchronizedArray<SubscriberCallbacks>()
    /// The `dropped` flag and the subscriber-link list live under a *single*
    /// lock so that "append a link unless dropped" and "mark dropped, then
    /// drain the links" are each one atomic critical section. When these were
    /// separate primitives, a link could be appended *after* a concurrent drop
    /// had already snapshotted the array, leaving it stranded — never torn down
    /// and never delivered messages.
    private let linkState = Mutex(LinkState())
    let latch = ManagedAtomic(false)
    /// Lock-free mirror of `linkState.dropped`, for advisory reads such as
    /// `validateHeader`. The authoritative gate is always `linkState`.
    let isDropped = ManagedAtomic(false)
    let lastMessage: Mutex<SerializedMessage?> = .init(nil)

    private struct LinkState {
        var dropped = false
        var links: [SubscriberLink] = []
    }
    
    init(name: String,
         datatype: String,
         md5sum: String,
         messageDefinition: String,
         latch: Bool) {
        
        self.name = name
        self.datatype = datatype
        self.md5sum = md5sum
        self.messageDefinition = messageDefinition
        self.latch.store(latch, ordering: .relaxed)
    }
    
    deinit {
        ROS_DEBUG("deinit called for Publication \(name)")
        dropPublication()
    }
    
    nonisolated var isLatched: Bool {
        return latch.load(ordering: .relaxed)
    }
    
    func addCallbacks(callback: SubscriberCallbacks) {
        pubCallbacks.append(callback)
        
        // Add connect callbacks for all current subscriptions if this publisher wants them
        if let connect = callback.connect {
            let links = linkState.withLock { $0.links }
            for link in links {
                _ = PeerConnDisconnCallback(callback: connect,
                                            subLink: link,
                                            useTrackedObject: callback.hasTrackedObject,
                                            trackedObject: callback.trackedObject)
                ROS_ERROR("addCallbacks logic not implemented")
            }
        }
    }
    
    func removeCallbacks(callback: SubscriberCallbacks) {
        pubCallbacks.removeAll(where: { callback === $0 })
    }
    
    func dropPublication() {
        dropAllConnections()
    }

    func addSubscriberLink(_ link: SubscriberLink) {
        // Gate the dropped-check and the append within a single critical
        // section: if a drop is concurrently in progress, either we observe
        // `dropped` and reject the link, or we append before the drop's drain
        // and the link is guaranteed to be torn down by that drain.
        let added = linkState.withLock { state -> Bool in
            guard !state.dropped else { return false }
            state.links.append(link)
            return true
        }

        guard added else { return }

        let last = lastMessage.withLock { $0 }
        if isLatched, let last, !last.buf.isEmpty {
            Task {
                await link.enqueueMessage(last)
                peerConnect(link: link)
            }
        } else {
            // This call invokes the subscribe callback if there is one.
            // This must happen *after* the push_back above, in case the
            // callback uses publish().
            peerConnect(link: link)
        }
    }

    func removeSubscriberLink(_ link: SubscriberLink) {
        let removed = linkState.withLock { state -> Bool in
            guard !state.dropped else { return false }
            guard let it = state.links.firstIndex(where: { $0.connectionId == link.connectionId }) else {
                return false
            }
            state.links.remove(at: it)
            return true
        }

        if removed {
            self.peerDisconnect(subLink: link)
        }
    }
    
    func getStats() -> XmlRpcValue {
        ROS_DEBUG("Publication::getStats")
        return .init(str: "Publication::getStats")
    }
    
    func getInfo() -> [XmlRpcValue] {
        var ret = [XmlRpcValue]()
        let links = linkState.withLock { $0.links }
        for sub in links {
            let info: [Any] = [
                sub.connectionId,
                sub.destinationCallerId,
                "o",
                "TCPROS",
                name,
                true,
                sub.transportInfo]
            ret.append(XmlRpcValue(anyArray: info))
        }

        return ret
    }

    func dropAllConnections() {
        // Atomically mark the publication dropped and snapshot-and-clear the
        // links, so no concurrent `addSubscriberLink` can slip a link past the
        // drain. Idempotent: a second drop observes `dropped` and no-ops.
        let removed = linkState.withLock { state -> [SubscriberLink] in
            guard !state.dropped else { return [] }
            state.dropped = true
            let links = state.links
            state.links.removeAll()
            return links
        }

        // Mirror the flag for lock-free advisory readers.
        isDropped.store(true, ordering: .relaxed)

        // Tear down outside the lock: `dropParentPublication()` calls back into
        // `removeSubscriberLink`, which now observes `dropped` and no-ops.
        for link in removed {
            link.dropParentPublication()
        }
    }
    
    func peerConnect(link: SubscriberLink) {
        pubCallbacks.forEach { cbs in
            if let conn = cbs.connect {
                let pcdc = PeerConnDisconnCallback(callback: conn,
                                                   subLink: link,
                                                   useTrackedObject: cbs.hasTrackedObject,
                                                   trackedObject: cbs.trackedObject)
                pcdc.call()
            }
        }
    }
    
    func peerDisconnect(subLink: SubscriberLink) {
        pubCallbacks.forEach {
            if let disconnect = $0.disconnect {
                let pcdc = PeerConnDisconnCallback(callback: disconnect,
                                                   subLink: subLink,
                                                   useTrackedObject: $0.hasTrackedObject,
                                                   trackedObject: $0.trackedObject)
                pcdc.call()
            }
            
        }
    }
    
    var numCallbacks: Int {
        return pubCallbacks.count
    }
    
    @discardableResult
    func incrementSequence() -> UInt32 {
        return sequenceNr.loadThenWrappingIncrement(ordering: .relaxed)
    }
    
    func getNumSubscribers() -> Int {
        linkState.withLock { $0.links.count }
    }

    func hasSubscribers() -> Bool {
        linkState.withLock { !$0.links.isEmpty }
    }

    func publish(msg: SerializedMessage) async {
        precondition(msg.message != nil)

        for link in linkState.withLock({ $0.links }) {
            await link.enqueueMessage(msg)
        }
        
        if isLatched {
            lastMessage.withLock {
                $0 = msg
            }
        }
    }
    
    nonisolated func validateHeader(header: Header, errorMsg: inout String) -> Bool {
        guard let md5sum = header["md5sum"],
              let topic = header["topic"],
              let callerid = header["callerid"] else {
            ROS_DEBUG("Header from subscriber did not have the required elements: md5sum, topic, callerid")
            return false
        }
        // Check whether the topic has been deleted from
        // advertised_topics through a call to unadvertise(), which could
        // have happened while we were waiting for the subscriber to
        // provide the md5sum.
        if isDropped.load(ordering: .relaxed) {
            errorMsg = "received a tcpros connection for a nonexistent topic [\(topic)] from [\(callerid)"
            ROS_DEBUG(errorMsg)
            return false
        }
        
        if self.md5sum != md5sum && md5sum != "*" && self.md5sum != "*" {
            let datatype = header["type"] ?? "no datatype"
            
            errorMsg = "Client [\(callerid)] wants topic \(topic)" +
            " to have datatype/md5sum [\(datatype)/\(md5sum)]" +
            ", but our version has [\(self.datatype)/\(self.md5sum)" +
            "]. Dropping connection."
            
            ROS_DEBUG(errorMsg)
            return false
        }
        
        return true
    }
    
}
