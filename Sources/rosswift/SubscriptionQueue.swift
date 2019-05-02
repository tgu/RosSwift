//
//  SubscriptionQueue.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-11-20.
//

import Deque
import Foundation
import RosTime


extension NSRecursiveLock {
    public func trySync<R>(execute work: () throws -> R) rethrows -> R? {
        guard `try`() else { return nil }
        defer { unlock() }
        return try work()
    }
}

internal final class SubscriptionQueue: CallbackInterface {
    internal struct Item {
        let helper: SubscriptionCallbackHelper
        let deserializer: MessageDeserializer
        let hasTrackedObject: Bool
        let trackedObject: AnyObject?
        let receiptTime: Time
    }

    private var queue: Deque<Item>
    private let topic: String
    private let size: UInt32
    private var wasFull: Bool
    let queueQueue = DispatchQueue(label: "queueQueue")
    let allowConcurrentCallbacks: Bool
    let callbackMutex = NSRecursiveLock()

    public init(topic: String, queueSize: UInt32, allowConcurrentCallbacks: Bool) {
        self.topic = topic
        self.size = queueSize
        self.wasFull = false
        self.allowConcurrentCallbacks = allowConcurrentCallbacks
        self.queue = Deque<Item>(minimumCapacity: queueSize == 0 ? 10 : Int(queueSize))
    }

    @discardableResult
    func push(helper: SubscriptionCallbackHelper,
              deserializer: MessageDeserializer,
              hasTrackedObject: Bool = false,
              trackedObject: AnyObject? = nil,
              receiptTime: Time = Time()) -> Bool {

        let item = Item(helper: helper,
                        deserializer: deserializer,
                        hasTrackedObject: hasTrackedObject,
                        trackedObject: trackedObject,
                        receiptTime: receiptTime)

        queueQueue.sync {
            if size > 0 && queue.count >= size {
                queue.popFirst()
                if !self.wasFull {
                    self.wasFull = true
                    ROS_DEBUG("Incoming queue was full for topic \"\(topic)\". Discarded oldest message (current queue size [\(queue.count)])")
                }
            } else {
                self.wasFull = false
            }

            queue.append(item)
        }

        return self.wasFull
    }

    func clear() {
        callbackMutex.sync {
            queueQueue.sync {
                queue.removeAll()
            }
        }
    }

    @discardableResult
    func call() -> CallResult {
        var ret = CallResult.tryAgain
        if allowConcurrentCallbacks {
            ret = call0()
        } else {
            callbackMutex.trySync {
                ret = call0()
            }
        }
        return ret
    }

    private func call0() -> CallResult {
        let info: Item? = queueQueue.sync {
            return queue.popFirst()
        }

        guard let item = info else {
            return .invalid
        }

        if item.hasTrackedObject {
            ROS_ERROR("tracker logic not implemented")
        }

        if let msg = item.deserializer.deserialize() {
            item.helper.call(msg: msg, item: item)
        }

        return .success
    }

    func ready() -> Bool {
        return true
    }

    func full() -> Bool {
        return queueQueue.sync {
            size > 0 && queue.count >= size
        }
    }

}
