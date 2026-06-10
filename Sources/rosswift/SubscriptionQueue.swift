//
//  SubscriptionQueue.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-11-20.
//

import DequeModule
import Foundation
import RosTime
import Synchronization
import Atomics

extension NSRecursiveLock {
    public func trySync<R>(execute work: () throws -> R) rethrows -> R? {
        guard `try`() else { return nil }
        defer { unlock() }
        return try work()
    }
}

extension NSLocking {
    @inline(__always)
    internal func sync<T>(_ closure: () -> T) -> T {
        self.lock()
        defer { self.unlock() }
        return closure()
    }
    
    @inline(__always)
    internal func sync(_ closure: () -> Void)  {
        self.lock()
        defer { self.unlock() }
        closure()
    }
    
}


internal actor SubscriptionQueue: AsyncCallbackInterface {
    internal struct Item: Sendable {
        let helper: SubscriptionCallbackHelper
        let deserializer: MessageDeserializer
        let hasTrackedObject: Bool
        weak var trackedObject: TrackableObject?
        let receiptTime: Time
    }

    /// True while the current task is dispatching a callback for this queue.
    /// Lets `clear()` distinguish an outside caller (which must wait for
    /// in-flight callbacks to finish) from a callback re-entering `clear()`
    /// on itself (which would otherwise deadlock waiting on its own count).
    @TaskLocal static var isDispatchingCallback: Bool = false

    private var queue: Deque<Item>
    private let topic: String
    private let size: UInt32
    private var wasFull = false
    let allowConcurrentCallbacks: Bool
    private var inFlightCalls = 0
    private var clearWaiters: [CheckedContinuation<Void, Never>] = []
    
    public init(topic: String, queueSize: UInt32, allowConcurrentCallbacks: Bool) {
        self.topic = topic
        self.size = queueSize
        self.allowConcurrentCallbacks = allowConcurrentCallbacks
        self.queue = Deque<Item>(minimumCapacity: queueSize == 0 ? 10 : Int(queueSize))
    }
    
    @discardableResult
    func push(helper: SubscriptionCallbackHelper,
              deserializer: consuming MessageDeserializer,
              hasTrackedObject: Bool = false,
              trackedObject: TrackableObject? = nil,
              receiptTime: Time = Time()) -> Bool {
        
        let item = Item(helper: helper,
                        deserializer: deserializer,
                        hasTrackedObject: hasTrackedObject,
                        trackedObject: trackedObject,
                        receiptTime: receiptTime)
        
        if size > 0 && queue.count >= size {
            _ = queue.popFirst()
            if !self.wasFull {
                self.wasFull = true
                ROS_DEBUG("Incoming queue was full for topic \"\(topic)\". Discarded oldest message (current queue size [\(queue.count)])")
            }
        } else {
            self.wasFull = false
        }
        
        queue.append(item)
        
        return self.wasFull
    }
    
    func clear() async {
        // Wait for any callbacks currently being dispatched to finish, so
        // callers can rely on the queue being quiescent on return. Skip the
        // wait if we're called from inside a callback for this queue,
        // otherwise we'd deadlock waiting on our own in-flight count.
        if !Self.isDispatchingCallback {
            while inFlightCalls > 0 {
                await withCheckedContinuation { cont in
                    clearWaiters.append(cont)
                }
            }
        }
        queue.removeAll()
    }

    @discardableResult
    func call() async throws -> CallResult {
        return try await call0()
    }

    private func callDidFinish() {
        inFlightCalls -= 1
        if inFlightCalls == 0 {
            let waiters = clearWaiters
            clearWaiters.removeAll()
            for w in waiters {
                w.resume()
            }
        }
    }

    private func call0() async throws -> CallResult {
        let info: Item? = queue.popFirst()

        guard let item = info else {
            return .invalid
        }

        if item.hasTrackedObject {
            if item.trackedObject == nil {
                ROS_DEBUG("tracker_object is nil")
                return .invalid
            }
        }

        inFlightCalls += 1
        defer { callDidFinish() }

        if let msg = await item.deserializer.deserialize() {
            try await Self.$isDispatchingCallback.withValue(true) {
                try await item.helper.call(msg: msg, item: item)
            }
        }

        return .success
    }
    
    var ready: Bool {
        return true
    }
    
    var full: Bool {
        size > 0 && queue.count >= size
    }
    
}
