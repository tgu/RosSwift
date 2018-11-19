//
//  Publisher.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import BinaryCoder
import Foundation
import StdMsgs

public protocol Publisher: class {
    func publish(message: Message)
}

    public final class SpecializedPublisher<M: Message>: Publisher {

        final class Impl<M: Message> {
            var topic: String
            var md5sum: String { return M.md5sum }
            var datatype: String { return M.datatype }
            var nodeHandle: Ros.NodeHandle?
            var callbacks: SubscriberCallbacks?
            var unadvertised: Bool

            init<M: Message>(topic: String, message: M.Type, node: Ros.NodeHandle, callbacks: SubscriberCallbacks) {
                self.topic = topic
                self.nodeHandle = node
                self.callbacks = callbacks
                self.unadvertised = false
            }

            deinit {
                unadvertise()
            }

            var isValid: Bool {
                return !unadvertised
            }

            func unadvertise() {
                ROS_DEBUG("Publisher on '\(topic)' deregistering callbacks.")
                if !unadvertised {
                    unadvertised = true
                    _ = Ros.TopicManager.instance.unadvertisePublisher(topic: topic, callbacks: callbacks)
                    nodeHandle = nil
                }
            }

        }

        var impl: Impl<M>?

        var topicManager: Ros.TopicManager {
            return Ros.TopicManager.instance
        }

        init(topic: String, message: M.Type, node: Ros.NodeHandle, callbacks: SubscriberCallbacks) {
            impl = Impl(topic: topic, message: message, node: node, callbacks: callbacks)
        }

        public func publish(message: Message) {
            guard let impl = impl else {
                ROS_ERROR("Call to publish() on an invalid Publisher")
                return
            }

            guard impl.isValid else {
                ROS_ERROR("Call to publish() on an invalid Publisher (topic [\(impl.topic)])")
                return
            }

            let md5sum = type(of: message).md5sum
            let datatype = type(of: message).datatype

            guard impl.md5sum == "*" || md5sum == "*" || impl.md5sum == md5sum else {
                fatalError("Trying to publish message of type [\(datatype)/\(md5sum)]" +
                    " on a publisher with type [\(impl.datatype)/\(impl.md5sum)]")
            }

            let msg = SerializedMessage(msg: message)

            topicManager.publish(topic: impl.topic, serMsg: msg)

        }

        func incrementSequence() {
            if let impl = impl, impl.isValid {
                topicManager.incrementSequence(topic: impl.topic)
            }
        }

        func shutdown() {
            if let impl = impl {
                impl.unadvertise()
                self.impl = nil
            }
        }

        func getNumSubscribers() -> Int {
            if let impl = impl, impl.isValid {
                return topicManager.getNumSubscribers(topic: impl.topic)
            }

            return 0
        }

        func isLatched() -> Bool {
            if let impl = impl, impl.isValid {
                if let pub = topicManager.lookupPublication(topic: impl.topic) {
                    return pub.isLatched()
                }
            }
            fatalError("Call to isLatched() on an invalid Publisher")
        }
    }
