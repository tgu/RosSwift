//
//  AdvertiseOptions.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//
import StdMsgs

public struct AdvertiseOptions<M : Message> {

    /// the topic to publish on

    public var topic: String

    /**
     Whether or not this publication should "latch".  A latching publication
     will automatically send out the last published message
     to any new subscribers.

    */
    public var latch: Bool
    public var connect_cb: SubscriberStatusCallback? = nil
    public var disconnect_cb: SubscriberStatusCallback? = nil
    weak var tracked_object: AnyObject?
    

    public init(topic: String, latch: Bool = false, _ messageType: M.Type) {
        self.topic = topic
        self.latch = latch
    }

}
