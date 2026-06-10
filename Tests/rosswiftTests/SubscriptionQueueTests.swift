//
//  SubscriptionQueueTests.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-11-23.
//

import Testing
@testable import RosSwift
import StdMsgs
import RosTime
import Foundation
import Synchronization

class SubscriptionQueueTests {


    @Test func testQueueSize() async throws {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 1, allowConcurrentCallbacks: false)

        #expect(await queue.full == false)

        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())

        await queue.push(helper: helper, deserializer: des)
        #expect(await queue.full )
        #expect(try await queue.call() == .success)
        #expect(await queue.full == false)
        await queue.push(helper: helper, deserializer: des)
        #expect(await queue.full )
        #expect(await queue.ready )
        await queue.push(helper: helper, deserializer: des)
        #expect(await queue.full )
        #expect(try await queue.call() == .success)
        #expect(try await queue.call() == .invalid)
        #expect(helper.calls == 2)
   }

    @Test func testInfiniteQueue() async throws {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: false)

        #expect(await queue.full == false)

        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())

        await queue.push(helper: helper, deserializer: des)
        #expect(try await queue.call() == .success)
        #expect(await queue.full == false)

        for _ in 0..<10000 {
            await queue.push(helper: helper, deserializer: des)
        }
        #expect(await queue.full == false)
        for _ in 0..<10000 {
            #expect(try await queue.call() == .success)
        }
        #expect(try await queue.call() == .invalid)
        #expect(helper.calls == 10001)
    }

    @Test func testClearCall() async throws {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: false)
        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())
        await queue.push(helper: helper, deserializer: des)
        await queue.clear()
        #expect(try await queue.call() == .invalid)
    }

    @Test func testClearThenCall() async throws {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: false)
        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())
        await queue.push(helper: helper, deserializer: des)
        await queue.clear()
        await queue.push(helper: helper, deserializer: des)
        #expect(try await queue.call() == .success)
    }

    struct ClearInCallback: CallbackProtocol {
        let queue: SubscriptionQueue

        func call() async {
            await queue.clear()
        }
    }

    @Test func testClearInCallback() async throws {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: false)
        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())

        helper.cb = ClearInCallback(queue: queue)
        await queue.push(helper: helper, deserializer: des)
        await queue.push(helper: helper, deserializer: des)
        await queue.push(helper: helper, deserializer: des)
        await queue.push(helper: helper, deserializer: des)
        #expect(try await queue.call() == .success)
        #expect(try await queue.call() == .invalid)
    }

    /// Async rendezvous for N parties. Each caller awaits `wait()`; the call
    /// completes once `count` parties have arrived. Replaces an earlier
    /// `NSCondition`-based barrier that parked threads on the cooperative
    /// pool and triggered priority-inversion warnings.
    actor Barrier {
        private var remaining: Int
        private var waiters: [CheckedContinuation<Void, Never>] = []

        init(count: Int) {
            self.remaining = count
        }

        func wait() async {
            remaining -= 1
            if remaining == 0 {
                let resume = waiters
                waiters.removeAll()
                for w in resume { w.resume() }
                return
            }
            await withCheckedContinuation { cont in
                waiters.append(cont)
            }
        }
    }


    final class ClearWhileThreadIsBlockingCallback: CallbackProtocol, @unchecked Sendable {
        var done: Bool
        let barrier: Barrier

        init(done: Bool, barrier: Barrier) {
            self.done = done
            self.barrier = barrier
        }

        func call() async {
            await barrier.wait()
            await WallDuration(milliseconds: 1000).sleep()
            done = true
        }
    }

    @Test func testClearWhileThreadIsBlocking() async {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: false)
        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())

        let barrier = Barrier(count: 2)

        let cb = ClearWhileThreadIsBlockingCallback(done: false, barrier: barrier)
        helper.cb = cb
        await queue.push(helper: helper, deserializer: des)
        let thread = Task.detached {
            try await queue.call()
        }
        await barrier.wait()
        await queue.clear()
        thread.cancel()
        #expect(cb.done)
    }

    final class WaitForBarrier: CallbackProtocol, @unchecked Sendable {
        let barrier: Barrier

        init(barrier: Barrier) {
            self.barrier = barrier
        }

        func call() async {
            await barrier.wait()
        }
    }


    @Test func testConcurrentCallbacks() async {
        #if false
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: true)
        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())
        let barrier = Barrier(count: 2)

        helper.cb = WaitForBarrier(barrier: barrier)
        let lock1 = NSConditionLock()
        let lock2 = NSConditionLock()
        await queue.push(helper: helper, deserializer: des)
        await queue.push(helper: helper, deserializer: des)
        let thread1 = Thread {
            lock1.lock()
            queue.call()
            lock1.unlock(withCondition: 1)
        }
        thread1.start()
        let thread2 = Thread {
            lock2.lock()
            queue.call()
            lock2.unlock(withCondition: 1)
        }
        thread2.start()
        lock1.lock(whenCondition: 1)
        lock2.lock(whenCondition: 1)
        #expect(helper.calls == 2)
        #endif
    }

    struct WaitForASecond: CallbackProtocol {
        func call() async throws {
            let now = ContinuousClock.now
            let next = now.advanced(by: .seconds(1))
            try await ContinuousClock().sleep(until: next)
        }
    }


    func testNonConcurrentOrdering() async {
        #if false
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: false)
        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())
        helper.cb = WaitForASecond()
        await queue.push(helper: helper, deserializer: des)
        await queue.push(helper: helper, deserializer: des)
        let lock1 = NSConditionLock()
        let lock2 = NSConditionLock()
        let thread1 = Thread {
            lock1.lock()
            queue.call()
            lock1.unlock(withCondition: 1)
        }
        thread1.start()
        let thread2 = Thread {
            lock2.lock()
            queue.call()
            lock2.unlock(withCondition: 1)
        }
        thread2.start()
        lock1.lock(whenCondition: 1)
        lock2.lock(whenCondition: 1)
        #expect(helper.calls == 1)
        queue.call()
        #expect(helper.calls == 2)
        #endif
    }
}

struct FakeMessage: Message {
    static let md5sum: String = ""
    static let datatype: String = ""
    static let definition: String = ""
}

protocol CallbackProtocol {
    func call() async throws
}


final class FakeSubHelper: SubscriptionCallbackHelper, CustomDebugStringConvertible, @unchecked Sendable {

    var debugDescription: String { return "Fake \(calls)" }

    let id = UUID()
    let _calls = Mutex(0)
    var cb : CallbackProtocol?

    var calls: Int {
        _calls.withLock { $0 }
    }

    func deserialize(data: [UInt8]) -> Message? {
        return FakeMessage()
    }

    func call(msg: Message, item: SubscriptionQueue.Item) async throws {
        _calls.withLock {
            $0 += 1
        }
        try await cb?.call()
    }

}

