//
//  Publisher.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import BinaryCoder
import StdMsgs
import Atomics

/// A handle for publishing messages on a topic.
///
/// Obtain one from `NodeHandle.advertise(topic:...)`. The publisher stays
/// advertised as long as the handle is retained; releasing it (or letting it
/// deinit) unadvertises the topic.
public protocol Publisher: Sendable {
    /// The resolved name of the topic this publisher advertises.
    var topic: String { get }

    /// The number of subscribers currently connected to this topic.
    var numSubscribers: Int { get }

    /// Publishes a message to all connected subscribers.
    ///
    /// The message type must match the type the topic was advertised with.
    /// - Parameter message: The message to send.
    func publish(message: Message) async
}

/// Concrete `Publisher` bound to a specific message type `M`.
///
/// Created by `NodeHandle.advertise`; you normally interact with it through
/// the `Publisher` protocol rather than referencing this type directly.
public final class SpecializedPublisher<M: Message>: Publisher {
    /// The resolved name of the topic this publisher advertises.
    public let topic: String
    var md5sum: String { return M.md5sum }
    var datatype: String { return M.datatype }
    let callbacks: SubscriberCallbacks
    private let unadvertised = ManagedAtomic(false)
    let topicManager: TopicManager
    
    internal init(topicManager: TopicManager, topic: String, message: M.Type, callbacks: SubscriberCallbacks) {
        self.topic = topic
        self.callbacks = callbacks
        self.topicManager = topicManager
    }
    
    deinit {
        unadvertise()
    }
    
    var isValid: Bool {
        return !unadvertised.load(ordering: .relaxed)
    }
    
    func unadvertise() {
        if unadvertised.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged {
            _ = topicManager.unadvertisePublisher(topic: topic, callbacks: callbacks)
            ROS_DEBUG("Publisher on '\(topic)' deregistering callbacks.")
        }
    }
    
    /// Publishes a message to all connected subscribers of `topic`.
    ///
    /// No-op (with an error log) if the publisher has been unadvertised.
    /// Traps if `message`'s type doesn't match the advertised type.
    /// - Parameter message: The message to send.
    public func publish(message: Message) async {
        guard isValid else {
            ROS_ERROR("Call to publish() on an invalid Publisher (topic [\(topic)])")
            return
        }
        
        let md5sum = type(of: message).md5sum
        let datatype = type(of: message).datatype
        
        guard self.md5sum == "*" || md5sum == "*" || self.md5sum == md5sum else {
            fatalError("Trying to publish message of type [\(datatype)/\(md5sum)]" +
                       " on a publisher with type [\(self.datatype)/\(self.md5sum)]")
        }
        await topicManager.publish(topic: topic, message: message)
    }
    
    func shutdown() {
        unadvertise()
    }
    
    /// The number of subscribers currently connected to this topic (0 if the
    /// publisher has been unadvertised).
    public var numSubscribers: Int {
        if isValid {
            return topicManager.getNumSubscribers(topic: topic)
        }
        
        return 0
    }
    
    var isLatched: Bool {
        if isValid {
            if let pub = topicManager.lookupPublication(topic: topic) {
                return pub.isLatched
            }
        }
        ROS_ERROR("Call to isLatched on an invalid Publisher")
        return false
    }
}
