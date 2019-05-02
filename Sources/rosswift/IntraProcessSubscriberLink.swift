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
    let isIntraprocess = true
    let transportInfo = "INTRAPROCESS"

    weak var subscriber: IntraProcessPublisherLink?
    var isDropped = Atomic<Bool>(value: false)

    init(parent: Publication) {
        self.parent = parent
        topic = parent.name
    }

    func setSubscriber(ros: Ros, subscriber: IntraProcessPublisherLink) {
        self.subscriber = subscriber
        connectionId = UInt(ros.connectionManager.getNewConnectionID())
        destinationCallerId = ros.name
    }

    func isLatching() -> Bool {
        return parent?.isLatching() ?? false
    }

    func enqueueMessage(m: SerializedMessage) {
        if isDropped.load() {
            return
        }

        // We have to remove the four first bytes with length information

        m.buf = [UInt8](m.buf.dropFirst(4))

        subscriber?.handleMessage(m: m)
    }

    func drop() {
        if isDropped.compareAndExchange(expected: false, desired: true) {
            subscriber?.drop()
            subscriber = nil

            ROS_DEBUG("Connection to local subscriber on topic [\(topic)] dropped")
            parent.removeSubscriberLink(self)
        }
    }



}
