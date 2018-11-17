//
//  Subscriber.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation
import StdMsgs
import RosTime

extension Ros {
public final class Subscriber {
    var topic: String
    var node: Ros.NodeHandle?
    var helper: SubscriptionCallbackHelper?
    var isUnsubscribed: Bool

    struct LatchInfo {
        let message: SerializedMessage
        let link: PublisherLink
        let connectionHeader: StringStringMap
        let receiptTime: RosTime.Time
    }

    var latchedMessages = [ObjectIdentifier: LatchInfo]()

    public init(topic: String, node: Ros.NodeHandle, helper: SubscriptionCallbackHelper?) {
        self.topic = topic
        self.node = node
        self.helper = helper
        self.isUnsubscribed = false
    }

    deinit {
        ROS_DEBUG("Subscriber on '\(self.topic)' deregistering callbacks.")
        unsubscribe()
    }

    func shutdown() {
        unsubscribe()
    }

    func unsubscribe() {
        if !isUnsubscribed {
            isUnsubscribed = true
            TopicManager.instance.unsubscribe(topic: topic, helper: helper!)
            node = nil
            helper = nil
            topic = ""
        }
    }

    func getTopic() -> String {
        return topic
    }

    func getNumPublishers() -> Int {
        if !isUnsubscribed {
            return TopicManager.instance.getNumPublishers(topic: topic)
        }
        return 0
    }

}

}
