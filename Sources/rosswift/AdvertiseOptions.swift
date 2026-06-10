//
//  AdvertiseOptions.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//
import StdMsgs

public struct AdvertiseOptions<M: Message>: Sendable {
    
    /// the topic to publish on
    
    public let topic: String
    
    /// The maximum number of outgoing messages to be queued for delivery to subscribers
    let queueSize: UInt
    
    /// Whether or not this publication should "latch".  A latching publication
    /// will automatically send out the last published message
    /// to any new subscribers.
    
    public let latch: Bool
    public let connectCallBack: SubscriberStatusCallback?
    public let disconnectCallBack: SubscriberStatusCallback?
    weak var trackedObject: TrackableObject?
    
    /// Queue to add callbacks to.  If NULL, the global callback queue will be used
    var callbackQueue: AsyncCallbackQueue?
    
    public init(topic: String,
                queueSize: UInt = 0,
                latch: Bool = false,
                _ messageType: M.Type,
                connectCall: SubscriberStatusCallback? = nil,
                disconnectCall: SubscriberStatusCallback? = nil,
                queue: AsyncCallbackQueue? = nil,
                tracked_object: TrackableObject? = nil
    ) {
        
        self.topic = topic
        self.latch = latch
        self.queueSize = queueSize
        self.connectCallBack = connectCall
        self.disconnectCallBack = disconnectCall
        self.callbackQueue = queue
        self.trackedObject = tracked_object
    }
    
    func set(topic: String) -> AdvertiseOptions<M> {
        AdvertiseOptions(topic: topic, queueSize: queueSize, latch: latch, M.self, connectCall: connectCallBack, disconnectCall: disconnectCallBack, queue: callbackQueue, tracked_object: trackedObject)
    }
    
    func set(callbackQueue: AsyncCallbackQueue) -> AdvertiseOptions<M> {
        AdvertiseOptions(topic: topic, queueSize: queueSize, latch: latch, M.self, connectCall: connectCallBack, disconnectCall: disconnectCallBack, queue: callbackQueue, tracked_object: trackedObject)
    }
    
}
