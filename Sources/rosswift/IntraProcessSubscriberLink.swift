//
//  IntraProcessSubscriberLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-23.
//

import Foundation
import NIOConcurrencyHelpers
import StdMsgs

final class IntraProcessSubscriberLink: SubscriberLink {
    weak var parent: Publication!
    var connectionId: UInt = 0
    var destinationCallerId: String = ""
    let topic: String

    weak var subscriber: IntraProcessPublisherLink?
    var isDropped = Atomic<Bool>(value: false)

    init(parent: Publication) {
        self.parent = parent
        topic = parent.name
    }

    func setSubscriber(subscriber: IntraProcessPublisherLink) {
        self.subscriber = subscriber
        connectionId = UInt(Ros.ConnectionManager.instance.getNewConnectionID())
        destinationCallerId = Ros.ThisNode.getName()
    }

    func isLatching() -> Bool {
        return parent?.isLatching() ?? false
    }

    func enqueueMessage(m: SerializedMessage, ser: Bool, nocopy: Bool) {
        if isDropped.load() {
            return
        }

        subscriber?.handleMessage(m: m, ser: ser, nocopy: nocopy)
    }

    func drop() {
        if isDropped.compareAndExchange(expected: false, desired: true) {
            subscriber?.drop()
            subscriber = nil

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
        if isDropped.load() {
            return
        }

        subscriber?.getPublishTypes(ser: &ser, nocopy: &nocopy, ti: ti)
    }

}
