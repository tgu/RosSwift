//
//  Subscriber.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//


/// Manages an subscription callback on a specific topic.
///
/// A Subscriber should always be created through a call to NodeHandle.subscribe(), or copied from one
/// that was. Once all copies of a specific
/// Subscriber go out of scope, the subscription callback associated with that handle will stop
/// being called.  Once all Subscriber for a given topic go out of scope the topic will be unsubscribed.


public final class Subscriber {
    public let topic: String
    public let node: NodeHandle

    /// Returns the number of publishers this subscriber is connected to

    public var publisherCount: Int {
        return node.ros.topicManager.getNumPublishers(topic: topic)
    }

    // internal

    internal let helper: SubscriptionCallbackHelper

    internal init(topic: String, node: NodeHandle, helper: SubscriptionCallbackHelper) {
        self.topic = topic
        self.node = node
        self.helper = helper
    }

    deinit {
        ROS_DEBUG("Subscriber on '\(self.topic)' deregistering callbacks.")
        _ = node.ros.topicManager.unsubscribe(topic: topic, helper: helper)
    }


}
