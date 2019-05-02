//
//  SubscribeOptions.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation
import StdMsgs

public struct SubscribeOptions<M: Message> {
    var topic: String
    let transportHints = TransportHints()
    let helper: SubscriptionCallbackHelper 
    var trackedObject: AnyObject?
    var allowConcurrentCallbacks = false
    var callbackQueue: CallbackQueueInterface
    let queueSize: UInt32

    init(topic: String, queueSize: UInt32, queue: CallbackQueueInterface, callback: @escaping ((M) -> Void)) {
        self.topic = topic
        self.helper = SubscriptionCallbackHelperT(callback: callback)
        self.queueSize = queueSize
        self.callbackQueue = queue
    }

    init(topic: String, queueSize: UInt32, queue: CallbackQueueInterface, callback: @escaping ((MessageEvent<M>) -> Void)) {
        self.topic = topic
        self.helper = SubscriptionEventCallbackHelperT(callback: callback)
        self.queueSize = queueSize
        self.callbackQueue = queue
    }

}
