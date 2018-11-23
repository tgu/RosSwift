//
//  AdvertiseOptions.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//
import StdMsgs

public struct AdvertiseOptions<M: Message> {

    /// the topic to publish on

    public var topic: String

    /// The maximum number of outgoing messages to be queued for delivery to subscribers
    let queueSize: UInt

    /// Whether or not this publication should "latch".  A latching publication
    /// will automatically send out the last published message
    /// to any new subscribers.

    public var latch: Bool
    public var connectCallBack: SubscriberStatusCallback?
    public var disconnectCallBack: SubscriberStatusCallback?
    weak var trackedObject: AnyObject?

    /// Queue to add callbacks to.  If NULL, the global callback queue will be used
    var callbackQueue: CallbackQueueInterface?

    public init(topic: String,
                queueSize: UInt = 0,
                latch: Bool = false,
                _ messageType: M.Type,
                connectCall: SubscriberStatusCallback? = nil,
                disconnectCall: SubscriberStatusCallback? = nil,
                queue: CallbackQueueInterface? = nil) {

        self.topic = topic
        self.latch = latch
        self.queueSize = queueSize
        self.connectCallBack = connectCall
        self.disconnectCallBack = disconnectCall
        self.callbackQueue = queue
    }

}
