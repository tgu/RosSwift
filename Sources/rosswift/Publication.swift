//
//  Publication.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-04.
//

import Foundation
import StdMsgs
import NIOConcurrencyHelpers

typealias VoidConstPtr = AnyObject?

class PeerConnDisconnCallback: CallbackInterface {

    var callback: SubscriberStatusCallback
    var sub_link : SubscriberLink!
    var use_tracked_object : Bool
    var tracked_object : VoidConstPtr
    

    init(callback: @escaping SubscriberStatusCallback, sub_link: SubscriberLink, use_tracked_object: Bool, tracked_object: VoidConstPtr) {
        self.callback = callback
        self.sub_link = sub_link
        self.use_tracked_object = use_tracked_object
        self.tracked_object = tracked_object
    }

    func call() -> CallResult {
        var tracker : VoidConstPtr = nil
        if use_tracked_object {
            tracker = tracked_object

            if tracker == nil {
                return .invalid
            }
        }

        let pub = SingleSubscriberPublisher(link: sub_link)
        callback(pub)

        return .success
    }

    func ready() -> Bool {
        return true
    }


}

class Publication {
    let name: String
    let datatype: String
    let md5sum: String
    let message_definition: String
    var sequenceNr = Atomic<UInt32>(value: 0)
    var intraprocess_subscriber_count_ = 0
    var pubCallbacks = [SubscriberCallbacks]()
    var subscriber_links = [SubscriberLink]()
    let latch : Bool
    let has_header : Bool
    var dropped = Atomic<Bool>(value: false)
    var intraprocess_subscriber_count : Int
    var publish_queue = [SerializedMessage]()
    var last_message : SerializedMessage?

    let callbacks_mutex_ = DispatchQueue(label: "callbacks_mutex_")
    let publish_queue_mutex_ = DispatchQueue(label: "publish_queue_mutex_")
    let subscriber_links_mutex_ = DispatchQueue(label: "subscriber_links_mutex_")

    init(name: String, datatype: String, md5sum: String,
         message_definition: String,
         latch: Bool,
         has_header: Bool) {

        self.name = name
        self.datatype = datatype
        self.md5sum = md5sum
        self.message_definition = message_definition
        self.latch = latch
        self.has_header = has_header
        self.intraprocess_subscriber_count = 0
    }

    deinit {
        ROS_DEBUG("deinit called for Publication \(name)")
        drop()
    }

    func isLatched() -> Bool {
        return latch
    }
    
    func addCallbacks(callback: SubscriberCallbacks) {
        callbacks_mutex_.sync {

            pubCallbacks.append(callback)

            // Add connect callbacks for all current subscriptions if this publisher wants them
            if let connect = callback.connect {
                subscriber_links_mutex_.sync {
                    subscriber_links.forEach {
                        let cb = PeerConnDisconnCallback(callback: connect, sub_link: $0, use_tracked_object: callback.has_tracked_object, tracked_object: callback.tracked_object)
//                        queue.addCallback(callback: cb, owner_id: cb.hash)
                    }
                }
            }

        }
    }

    func removeCallbacks(callback: SubscriberCallbacks) {
        #if swift(>=4.2)
        callbacks_mutex_.sync {
            pubCallbacks.removeAll(where: {callback === $0})
        }
        #else
        callbacks_mutex_.sync {
            if let index = pubCallbacks.index(where: {callback === $0}) {
                pubCallbacks.remove(at: index)
            }
        }
        #endif
    }


    func drop() {
        if dropped.compareAndExchange(expected: false, desired: true) {
            dropAllConnections()
        }
    }

    func enqueueMessage(m: SerializedMessage) -> Bool {
        if dropped.load() {
            return false
        }

        precondition(!m.buf.isEmpty)

        subscriber_links_mutex_.sync {
            let _ = incrementSequence()

            if has_header {
                ROS_ERROR("header not handled")
            }
            subscriber_links.forEach {
                $0.enqueueMessage(m: m, ser: true, nocopy: false)
            }


            if latch {
                last_message = m
            }
        }
        return true;

    }


    func addSubscriberLink(_ link: SubscriberLink) {
        if dropped.load() {
            return
        }

        subscriber_links_mutex_.sync {

            subscriber_links.append(link)

            if link.isIntraprocess() {
                intraprocess_subscriber_count += 1
            }

            if latch, let lm = last_message, lm.buf.isEmpty  {
                link.enqueueMessage(m: lm, ser: true, nocopy: true)
            }

            // This call invokes the subscribe callback if there is one.
            // This must happen *after* the push_back above, in case the
            // callback uses publish().
            peerConnect(link: link)
        }
    }

    func removeSubscriberLink(_ link: SubscriberLink) {
        if dropped.load() {
            ROS_DEBUG("is dropped")
            return
        }

        var subLink : SubscriberLink? = nil

        subscriber_links_mutex_.async {
            if link.isIntraprocess() {
                self.intraprocess_subscriber_count_ -= 1
            }

            if let it = self.subscriber_links.index(where: { $0 === link }) {
                subLink = self.subscriber_links[it]
                self.subscriber_links.remove(at: it)
            }
        }

        if let subLink = subLink {
            self.peerDisconnect(sub_link: subLink)
        }
    }

    func getStats() -> XmlRpcValue {
        ROS_DEBUG("Publication::getStats")
        return .init(str: "Publication::getStats")
    }

    func getInfo(info: inout XmlRpcValue) {
        ROS_DEBUG("Publication::getInfo")
    }

    func dropAllConnections() {
        var local_publishers = [SubscriberLink]()
        subscriber_links_mutex_.sync {
            swap(&local_publishers, &subscriber_links)
        }
        local_publishers.forEach { $0.drop() }
    }

    func peerConnect(link: SubscriberLink) {
        pubCallbacks.forEach { (cbs) in
            if let conn = cbs.connect {
                let cb = PeerConnDisconnCallback(callback: conn, sub_link: link, use_tracked_object: cbs.has_tracked_object, tracked_object: cbs.tracked_object)
                DispatchQueue(label: "peerConnect").async {
                    cb.call()
                }
            }
        }
    }

    func peerDisconnect(sub_link: SubscriberLink) {
        pubCallbacks.forEach {
            if let disconnect = $0.disconnect {
                let cb = PeerConnDisconnCallback(callback: disconnect, sub_link: sub_link, use_tracked_object: $0.has_tracked_object, tracked_object: $0.tracked_object)
                DispatchQueue(label: "peerConnect").async {
                    cb.call()
                }
            }

        }
    }

    var numCallbacks: Int {
        return pubCallbacks.count
    }

    func incrementSequence() -> UInt32 {
        return sequenceNr.add(1)
    }

    func getNumSubscribers() -> Int {
        return subscriber_links_mutex_.sync(execute: {return subscriber_links.count})
    }

    func getPublishTypes(serialize: inout Bool, nocopy: inout Bool, ti: TypeInfo) {
        subscriber_links_mutex_.sync {
            subscriber_links.forEach({ (sub) in
                var s = false
                var n = false
                sub.getPublishTypes(ser: &s, nocopy: &n, ti: ti)
                serialize = serialize || s
                nocopy = nocopy || n
                if serialize && nocopy {
                    return
                }
            })
        }

    }

    func hasSubscribers() -> Bool {
        return subscriber_links_mutex_.sync(execute: {return !subscriber_links.isEmpty})
    }

    func publish(msg: SerializedMessage) {
        if msg.message != nil {
            subscriber_links_mutex_.sync {
                subscriber_links.forEach {
                    if $0.isIntraprocess() {
                        $0.enqueueMessage(m: msg, ser: false, nocopy: true)
                    }
                }
            }

            msg.message = nil
        }

        if msg.buf.count > 0 {
            _ = enqueueMessage(m: msg)
        }
    }

//    func processPublishQueue() {
//        var queue = [SerializedMessage]()
//        publish_queue_mutex_.sync {
//            if !dropped {
//                swap(&queue,&publish_queue)
//            }
//        }
//
//        queue.forEach {
//            let _ = enqueueMessage(m: $0)
//        }
//
//    }

    func isLatching() -> Bool {
        return latch
    }

    func validateHeader(header: Header, error_msg: inout String) -> Bool {
        guard let md5sum = header.getValue(key: "md5sum"),
            let topic = header.getValue(key: "topic"),
            let callerid = header.getValue(key: "callerid") else {
                let  error_msg = "Header from subscriber did not have the required elements: md5sum, topic, callerid"
                ROS_DEBUG(error_msg)
                return false
        }
        // Check whether the topic has been deleted from
        // advertised_topics through a call to unadvertise(), which could
        // have happened while we were waiting for the subscriber to
        // provide the md5sum.
        if dropped.load() {
            error_msg = "received a tcpros connection for a nonexistent topic [\(topic)] from [\(callerid)"
            ROS_DEBUG(error_msg)
            return false
        }

        if self.md5sum != md5sum && md5sum != "*" && self.md5sum != "*" {
            let datatype = header.getValue(key: "type") ?? "no datatype"

            error_msg = "Client [\(callerid)] wants topic \(topic)" +
                        " to have datatype/md5sum [\(datatype)/\(md5sum)]" +
                        ", but our version has [\(self.datatype)/\(self.md5sum)" +
                        "]. Dropping connection."

            ROS_DEBUG(error_msg)
            return false
        }

        return true
    }




}

