//
//  Publisher.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import BinaryCoder
import StdMsgs

public protocol Publisher {
    func publish(message: Message)
}

public final class SpecializedPublisher<M: Message>: Publisher {
    let topic: String
    var md5sum: String { return M.md5sum }
    var datatype: String { return M.datatype }
    let callbacks: SubscriberCallbacks
    private var unadvertised: Bool
    let topicManager: TopicManager

    internal init(topicManager: TopicManager, topic: String, message: M.Type, callbacks: SubscriberCallbacks) {
        self.topic = topic
        self.callbacks = callbacks
        self.unadvertised = false
        self.topicManager = topicManager
    }

    deinit {
        unadvertise()
    }

    var isValid: Bool {
        return !unadvertised
    }

    func unadvertise() {
        if !unadvertised {
            unadvertised = true
            _ = topicManager.unadvertisePublisher(topic: topic, callbacks: callbacks)
            ROS_DEBUG("Publisher on '\(topic)' deregistering callbacks.")
        }
    }

    public func publish(message: Message) {
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

        let msg = SerializedMessage(msg: message)
        topicManager.publish(topic: topic, serMsg: msg)
    }

    func incrementSequence() {
        if isValid {
            topicManager.incrementSequence(topic: topic)
        }
    }

    func shutdown() {
        unadvertise()
    }

    var numSubscribers: Int {
        if isValid {
            return topicManager.getNumSubscribers(topic: topic)
        }

        return 0
    }

    var isLatched: Bool {
        if isValid {
            if let pub = topicManager.lookupPublication(topic: topic) {
                return pub.isLatched()
            }
        }
        ROS_ERROR("Call to isLatched on an invalid Publisher")
        return false
    }
}
