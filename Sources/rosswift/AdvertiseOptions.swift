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

    /**
     Whether or not this publication should "latch".  A latching publication
     will automatically send out the last published message
     to any new subscribers.

    */
    public var latch: Bool
    public var connectCallBack: SubscriberStatusCallback?
    public var disconnectCallBack: SubscriberStatusCallback?
    weak var trackedObject: AnyObject?

    public init(topic: String, latch: Bool = false, _ messageType: M.Type) {
        self.topic = topic
        self.latch = latch
    }

}
