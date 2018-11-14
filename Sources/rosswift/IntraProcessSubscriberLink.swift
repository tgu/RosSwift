//
//  IntraProcessSubscriberLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-23.
//

import Foundation
import StdMsgs
import NIOConcurrencyHelpers


final class IntraProcessSubscriberLink: SubscriberLink {
    weak var parent : Publication!
    var connection_id : UInt = 0
    var destination_caller_id  : String = ""
    let topic : String

    weak var subscriber_ : IntraProcessPublisherLink?
    var dropped_ = Atomic<Bool>(value: false)

    init(parent: Publication) {
        self.parent = parent
        topic = parent.name
    }

    func setSubscriber(subscriber: IntraProcessPublisherLink) {
        subscriber_ = subscriber
        connection_id = UInt(Ros.ConnectionManager.instance.getNewConnectionID())
        destination_caller_id = Ros.this_node.getName()
    }

    func isLatching() -> Bool {
        return parent?.isLatching() ?? false
    }

    func enqueueMessage(m: SerializedMessage, ser: Bool, nocopy: Bool) {
        if dropped_.load() {
            return
        }

        subscriber_?.handleMessage(m: m, ser: ser, nocopy: nocopy)
    }

    func drop() {
        if dropped_.compareAndExchange(expected: false, desired: true){
            subscriber_?.drop()
            subscriber_ = nil

            ROS_DEBUG("Connection to local subscriber on topic [\(topic)] dropped")
            parent.removeSubscriberLink(self)
        }
    }

    func getTransportType() -> String {
        return "INTRAPROCESS"
    }

    func getTransportInfo() -> String {
        return getTransportType()
    }

    func isIntraprocess() -> Bool {
        return true
    }

    func getPublishTypes(ser: inout Bool, nocopy: inout Bool, ti: TypeInfo) {
        if dropped_.load() {
            return
        }

        subscriber_?.getPublishTypes(ser: &ser, nocopy: &nocopy, ti: ti)
    }
    
}
