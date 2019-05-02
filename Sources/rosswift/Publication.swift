//
//  Publication.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-04.
//

import Foundation
import NIOConcurrencyHelpers
import StdMsgs

internal final class PeerConnDisconnCallback: CallbackInterface {

    var callback: SubscriberStatusCallback
    var subLink: SubscriberLink!
    var useTrackedObject: Bool
    var trackedObject: AnyObject?

    init(callback: @escaping SubscriberStatusCallback,
         subLink: SubscriberLink,
         useTrackedObject: Bool = false,
         trackedObject: AnyObject? = nil) {

        self.callback = callback
        self.subLink = subLink
        self.useTrackedObject = useTrackedObject
        self.trackedObject = trackedObject
    }

    @discardableResult
    func call() -> CallResult {
        var tracker: AnyObject?
        if useTrackedObject {
            tracker = trackedObject

            if tracker == nil {
                return .invalid
            }
        }

        let pub = SingleSubscriberPublisher(link: subLink)
        callback(pub)

        return .success
    }

    func ready() -> Bool {
        return true
    }

}

final class Publication {
    let name: String
    let datatype: String
    let md5sum: String
    let messageDefinition: String
    var sequenceNr = Atomic<UInt32>(value: 0)
    var intraprocessSubscriberCount = 0
    var pubCallbacks = [SubscriberCallbacks]()
    var subscriberLinks = [SubscriberLink]()
    let latch: Bool
    let hasHeader: Bool
    var isDropped = Atomic<Bool>(value: false)
    var lastMessage: SerializedMessage?

    let callbacksQueue = DispatchQueue(label: "callbacksQueue")
    let subscriberLinksQueue = DispatchQueue(label: "subscriberLinksQueue")

    init(name: String,
         datatype: String,
         md5sum: String,
         messageDefinition: String,
         latch: Bool,
         hasHeader: Bool) {

        self.name = name
        self.datatype = datatype
        self.md5sum = md5sum
        self.messageDefinition = messageDefinition
        self.latch = latch
        self.hasHeader = hasHeader
    }

    deinit {
        ROS_DEBUG("deinit called for Publication \(name)")
        drop()
    }

    func isLatched() -> Bool {
        return latch
    }

    func addCallbacks(callback: SubscriberCallbacks) {
        callbacksQueue.sync {

            pubCallbacks.append(callback)

            // Add connect callbacks for all current subscriptions if this publisher wants them
            if let connect = callback.connect {
                subscriberLinksQueue.sync {
                    subscriberLinks.forEach {
                        _ = PeerConnDisconnCallback(callback: connect,
                                                    subLink: $0,
                                                    useTrackedObject: callback.hasTrackedObject,
                                                    trackedObject: callback.trackedObject)
                        ROS_ERROR("addCallbacks logic not implemented")
                    }
                }
            }

        }
    }

    func removeCallbacks(callback: SubscriberCallbacks) {
        #if swift(>=4.2)
        callbacksQueue.sync {
            pubCallbacks.removeAll(where: { callback === $0 })
        }
        #else
        callbacksQueue.sync {
            if let index = pubCallbacks.index(where: { callback === $0 }) {
                pubCallbacks.remove(at: index)
            }
        }
        #endif
    }

    func drop() {
        if isDropped.compareAndExchange(expected: false, desired: true) {
            dropAllConnections()
        }
    }

    func enqueueMessage(m: SerializedMessage) -> Bool {
        if isDropped.load() {
            return false
        }

        precondition(!m.buf.isEmpty)

        subscriberLinksQueue.sync {
            _ = incrementSequence()

            if hasHeader {
                ROS_ERROR("header not handled")
            }
            subscriberLinks.forEach {
                $0.enqueueMessage(m: m)
            }

            if latch {
                lastMessage = m
            }
        }
        return true

    }

    func addSubscriberLink(_ link: SubscriberLink) {
        if isDropped.load() {
            return
        }

        subscriberLinksQueue.sync {

            subscriberLinks.append(link)

            if link.isIntraprocess {
                intraprocessSubscriberCount += 1
            }

            if latch, let last = lastMessage, last.buf.isEmpty {
                link.enqueueMessage(m: last)
            }

            // This call invokes the subscribe callback if there is one.
            // This must happen *after* the push_back above, in case the
            // callback uses publish().
            peerConnect(link: link)
        }
    }

    func removeSubscriberLink(_ link: SubscriberLink) {
        if isDropped.load() {
            return
        }

        subscriberLinksQueue.async {
            if link.isIntraprocess {
                self.intraprocessSubscriberCount -= 1
            }

            if let it = self.subscriberLinks.firstIndex(where: { $0 === link }) {
                self.peerDisconnect(subLink: link)
                self.subscriberLinks.remove(at: it)
            }
        }
    }

    func getStats() -> XmlRpcValue {
        ROS_DEBUG("Publication::getStats")
        return .init(str: "Publication::getStats")
    }

    func getInfo() -> [XmlRpcValue] {
        var ret = [XmlRpcValue]()
        subscriberLinksQueue.sync {
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
        }

        return ret
    }

    func dropAllConnections() {
        var localPublishers = [SubscriberLink]()
        subscriberLinksQueue.sync {
            swap(&localPublishers, &subscriberLinks)
        }
        localPublishers.forEach { $0.drop() }
    }

    func peerConnect(link: SubscriberLink) {
        pubCallbacks.forEach { cbs in
            if let conn = cbs.connect {
                let pcdc = PeerConnDisconnCallback(callback: conn,
                                                   subLink: link,
                                                   useTrackedObject: cbs.hasTrackedObject,
                                                   trackedObject: cbs.trackedObject)
                DispatchQueue(label: "peerConnect").async {
                    pcdc.call()
                }
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
        return sequenceNr.add(1)
    }

    func getNumSubscribers() -> Int {
        return subscriberLinksQueue.sync(execute: { subscriberLinks.count })
    }

    func hasSubscribers() -> Bool {
        return subscriberLinksQueue.sync { !subscriberLinks.isEmpty }
    }

    func publish(msg: SerializedMessage) {
        precondition(msg.message != nil)

        incrementSequence()
        subscriberLinksQueue.sync {
            subscriberLinks.forEach {
                $0.enqueueMessage(m: msg)
            }
        }
    }

    func isLatching() -> Bool {
        return latch
    }

    func validateHeader(header: Header, errorMsg: inout String) -> Bool {
        guard let md5sum = header.getValue(key: "md5sum"),
            let topic = header.getValue(key: "topic"),
            let callerid = header.getValue(key: "callerid") else {
                ROS_DEBUG("Header from subscriber did not have the required elements: md5sum, topic, callerid")
                return false
        }
        // Check whether the topic has been deleted from
        // advertised_topics through a call to unadvertise(), which could
        // have happened while we were waiting for the subscriber to
        // provide the md5sum.
        if isDropped.load() {
            errorMsg = "received a tcpros connection for a nonexistent topic [\(topic)] from [\(callerid)"
            ROS_DEBUG(errorMsg)
            return false
        }

        if self.md5sum != md5sum && md5sum != "*" && self.md5sum != "*" {
            let datatype = header.getValue(key: "type") ?? "no datatype"

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
