//
//  SubscribeOptions.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import StdMsgs

public struct SubscribeOptions<M: Message>: Sendable {
    var topic: String
    let transportHints = TransportHints()
    let helper: SubscriptionCallbackHelper 
    weak var trackedObject: TrackableObject?
    var allowConcurrentCallbacks = false
    var callbackQueue: AsyncCallbackQueue
    let queueSize: UInt32
    
    init(topic: String, queueSize: UInt32, queue: AsyncCallbackQueue, trackedObject: TrackableObject? = nil, callback: @escaping @Sendable (M) -> Void) {
        self.topic = topic
        self.helper = SubscriptionCallbackHelperT(callback: callback)
        self.queueSize = queueSize
        self.callbackQueue = queue
        self.trackedObject = trackedObject
    }
    
    init(topic: String, queueSize: UInt32, queue: AsyncCallbackQueue, callback: @escaping @Sendable (MessageEvent<M>) -> Void) {
        self.topic = topic
        self.helper = SubscriptionEventCallbackHelperT(callback: callback)
        self.queueSize = queueSize
        self.callbackQueue = queue
    }
    
}
