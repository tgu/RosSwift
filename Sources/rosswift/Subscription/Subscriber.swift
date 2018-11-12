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
public class Subscriber {
    var topic_ : String
    var node_handle_ : Ros.NodeHandle?
    var helper_ : SubscriptionCallbackHelper?
    var unsubscribed_ : Bool

    struct LatchInfo {
        let message: SerializedMessage
        let link: PublisherLink
        let connection_header : M_string
        let receipt_time : RosTime.Time
    }

    var latched_messages_ = [ObjectIdentifier : LatchInfo]()


    public init(topic: String, node_handle: Ros.NodeHandle, helper: SubscriptionCallbackHelper?) {
        self.topic_ = topic
        self.node_handle_ = node_handle
        self.helper_ = helper
        self.unsubscribed_ = false
    }

    deinit {
        ROS_DEBUG("Subscriber on '\(self.topic_)' deregistering callbacks.")
        unsubscribe()
    }

    func shutdown() {
        unsubscribe()
    }

    func unsubscribe() {
        if !unsubscribed_ {
            unsubscribed_ = true
            TopicManager.instance.unsubscribe(topic: topic_, helper: helper_!)
            node_handle_ = nil
            helper_ = nil
            topic_ = ""
        }
    }

    func getTopic() -> String {
        return topic_
    }

    func getNumPublishers() -> Int {
        if !unsubscribed_ {
            return TopicManager.instance.getNumPublishers(topic: topic_)
        }
        return 0
    }

}

}
