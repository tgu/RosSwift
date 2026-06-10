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
    let subscriberLinks = SynchronizedArray<SubscriberLink>()
    let latch = ManagedAtomic(false)
    let isDropped = ManagedAtomic(false)
    let lastMessage: Mutex<SerializedMessage?> = .init(nil)
    
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
            subscriberLinks.forEach {
                _ = PeerConnDisconnCallback(callback: connect,
                                            subLink: $0,
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
        if isDropped.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged {
            dropAllConnections()
        }
    }
    
    func addSubscriberLink(_ link: SubscriberLink) {
        if isDropped.load(ordering: .relaxed) {
            return
        }
        
        
        subscriberLinks.append(link)
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
        if isDropped.load(ordering: .relaxed) {
            return
        }
        
        if let it = self.subscriberLinks.firstIndex(where: { $0.connectionId == link.connectionId }) {
            self.peerDisconnect(subLink: link)
            self.subscriberLinks.remove(at: it)
        }
    }
    
    func getStats() -> XmlRpcValue {
        ROS_DEBUG("Publication::getStats")
        return .init(str: "Publication::getStats")
    }
    
    func getInfo() -> [XmlRpcValue] {
        var ret = [XmlRpcValue]()
        subscriberLinks.forEach({ sub in
            let info: [Any] = [
                sub.connectionId,
                sub.destinationCallerId,
                "o",
                "TCPROS",
                name,
                true,
                sub.transportInfo]
            ret.append(XmlRpcValue(anyArray: info))
        })
        
        return ret
    }
    
    func dropAllConnections() {
        let localPublishers = subscriberLinks.all()
        subscriberLinks.removeAll()
        for pub in localPublishers {
            pub.dropParentPublication()
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
        subscriberLinks.count
    }
    
    func hasSubscribers() -> Bool {
        !subscriberLinks.isEmpty
    }
    
    func publish(msg: SerializedMessage) async {
        precondition(msg.message != nil)
        
        for link in subscriberLinks.all() {
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
