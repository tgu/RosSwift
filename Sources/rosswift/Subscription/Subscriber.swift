//
//  Subscriber.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Atomics

/// Manages an subscription callback on a specific topic.
///
/// A Subscriber should always be created through a call to `NodeHandle.subscribe(...)`, or copied from one
/// that was. Once all copies of a specific
/// Subscriber go out of scope, the subscription callback associated with that handle will stop
/// being called.  Once all Subscriber for a given topic go out of scope the topic will be unsubscribed.


public final class Subscriber {
    /// The resolved name of the subscribed topic.
    public let topic: String
    weak var node: NodeHandle?
    let unsubscribed = ManagedAtomic(false)
    internal let helper: SubscriptionCallbackHelper
    
    
    var isValid: Bool {
        !unsubscribed.load(ordering: .relaxed)
    }
    
    /// Returns the number of publishers this subscriber is connected to
    
    public var publisherCount: Int {
        if isValid, let node {
            return node.ros.topicManager.getNumPublishers(topic: topic)
        } else {
            return 0
        }
    }
    
    // internal
    
    internal init(topic: String, node: NodeHandle, helper: SubscriptionCallbackHelper) {
        self.topic = topic
        self.node = node
        self.helper = helper
    }
    
    func shutdown() {
        unsubscribe()
    }
    
    /// Cancels this subscription's callback.
    ///
    /// Idempotent. When the last `Subscriber` for a topic unsubscribes (or
    /// deinits), the node unsubscribes from the topic on the master.
    public func unsubscribe() {
        if !unsubscribed.compareExchange(expected: false, desired: true, ordering: .relaxed).original {
            if let node {
                let manager = node.ros.topicManager
                let topic = self.topic
                let helper = self.helper
                _ = manager.unsubscribe(topic: topic, helper: helper)
            }
            node = nil
        }
    }
    
    deinit {
        unsubscribe()
    }
    
    
}
