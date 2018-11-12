//
//  Publisher.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation
import StdMsgs
import BinaryCoder


public protocol Publisher: class {
    func publish(message: Message)
}

extension Ros {

    public class SpecializedPublisher<M: Message>: Publisher {

        class Impl<M : Message> {
            var topic: String
            var md5sum: String { return M.md5sum }
            var datatype: String { return M.datatype }
            var nodeHandle: Ros.NodeHandle?
            var callbacks: SubscriberCallbacks?
            var unadvertised: Bool

            init<M: Message>(topic: String, message: M.Type, node_handle: NodeHandle, callbacks: SubscriberCallbacks) {
                self.topic = topic
                self.nodeHandle = node_handle
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
                    let _ = TopicManager.instance.unadvertisePublisher(topic: topic, callbacks: callbacks)
                    nodeHandle = nil
                }
            }

        }

        var impl_ : Impl<M>?

        var topicManager: TopicManager {
            return TopicManager.instance
        }

        init(topic: String, message: M.Type, node_handle: NodeHandle, callbacks: SubscriberCallbacks) {
            impl_ = Impl(topic: topic, message: message, node_handle: node_handle, callbacks: callbacks)
        }

        public func publish(message: Message) {
            guard let impl = impl_ else {
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
                ROS_ERROR("Trying to publish message of type [\(datatype)/\(md5sum)] on a publisher with type [\(impl.datatype)/\(impl.md5sum)]")
                fatalError()
            }

            let msg = SerializedMessage(msg: message)

            topicManager.publish(topic: impl.topic, ser_msg: msg)

        }


        func incrementSequence() {
            if let impl = impl_, impl.isValid {
                topicManager.incrementSequence(topic: impl.topic)
            }
        }

        func shutdown() {
            if let impl = impl_ {
                impl.unadvertise()
                impl_ = nil
            }
        }

        func getNumSubscribers() -> Int  {
            if let impl = impl_, impl.isValid {
                return topicManager.getNumSubscribers(topic: impl.topic)
            }

            return 0
        }

        func isLatched() -> Bool {
            if let impl = impl_, impl.isValid {
                if let pub = topicManager.lookupPublication(topic: impl.topic) {
                    return pub.isLatched()
                }
            }
            fatalError("Call to isLatched() on an invalid Publisher")
        }
    }
}
