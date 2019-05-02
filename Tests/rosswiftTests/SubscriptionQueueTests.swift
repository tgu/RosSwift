//
//  SubscriptionQueueTests.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-11-23.
//

import XCTest
@testable import RosSwift
import StdMsgs
import RosTime

class SubscriptionQueueTests: XCTestCase {

    override func setUp() {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }

    override func tearDown() {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }

    func testQueueSize() {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 1, allowConcurrentCallbacks: false)

        XCTAssertFalse(queue.full())

        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())

        queue.push(helper: helper, deserializer: des)
        XCTAssert( queue.full() )
        XCTAssertEqual(queue.call(), .success)
        XCTAssertFalse(queue.full())
        queue.push(helper: helper, deserializer: des)
        XCTAssert( queue.full() )
        XCTAssert( queue.ready() )
        queue.push(helper: helper, deserializer: des)
        XCTAssert( queue.full() )
        XCTAssertEqual(queue.call(), .success)
        XCTAssertEqual(queue.call(), .invalid)
        XCTAssertEqual(helper.calls, 2)
   }

    func testInfiniteQueue() {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: false)

        XCTAssertFalse(queue.full())

        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())

        queue.push(helper: helper, deserializer: des)
        XCTAssertEqual(queue.call(), .success)
        XCTAssertFalse(queue.full())

        for _ in 0..<10000 {
            queue.push(helper: helper, deserializer: des)
        }
        XCTAssertFalse(queue.full())
        for _ in 0..<10000 {
            XCTAssertEqual(queue.call(), .success)
        }
        XCTAssertEqual(queue.call(), .invalid)
        XCTAssertEqual(helper.calls, 10001)
    }

    func testClearCall() {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: false)
        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())
        queue.push(helper: helper, deserializer: des)
        queue.clear()
        XCTAssertEqual(queue.call(), .invalid)
    }

    func testClearThenCall() {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: false)
        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())
        queue.push(helper: helper, deserializer: des)
        queue.clear()
        queue.push(helper: helper, deserializer: des)
        XCTAssertEqual(queue.call(), .success)
    }

    struct ClearInCallback: CallbackProtocol {
        let queue: SubscriptionQueue

        func call() {
            queue.clear()
        }
    }

    func testClearInCallback() {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: false)
        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())

        helper.cb = ClearInCallback(queue: queue)
        queue.push(helper: helper, deserializer: des)
        queue.push(helper: helper, deserializer: des)
        queue.push(helper: helper, deserializer: des)
        queue.push(helper: helper, deserializer: des)
        XCTAssertEqual(queue.call(), .success)
        XCTAssertEqual(queue.call(), .invalid)
    }

    final class Barrier {
        let condition = NSCondition()
        let mutex = DispatchQueue(label: "barrier")
        var count : UInt

        init(count: UInt) {
            self.count = count
        }

        func wait() {
            condition.lock()
            count -= 1
            if count == 0 {
                condition.broadcast()
            } else {
                condition.wait()
            }
            condition.unlock()
        }
    }


    class ClearWhileThreadIsBlockingCallback: CallbackProtocol {
        var done: Bool
        var barrier: Barrier

        init(done: Bool, barrier: Barrier) {
            self.done = done
            self.barrier = barrier
        }

        func call() {
            barrier.wait()
            WallDuration(milliseconds: 1000).sleep()
           done = true
        }
    }

    func testClearWhileThreadIsBlocking() {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: false)
        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())

        let barrier = Barrier(count: 2)

        let cb = ClearWhileThreadIsBlockingCallback(done: false, barrier: barrier)
        helper.cb = cb
        queue.push(helper: helper, deserializer: des)
        let thread = Thread {
            queue.call()
        }
        thread.start()
        barrier.wait()
        queue.clear()
        XCTAssert(cb.done)
    }

    class WaitForBarrier: CallbackProtocol {
        var barrier: Barrier

        init(barrier: Barrier) {
            self.barrier = barrier
        }

        func call() {
            barrier.wait()
        }
    }


    func testConcurrentCallbacks() {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: true)
        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())
        let barrier = Barrier(count: 2)

        helper.cb = WaitForBarrier(barrier: barrier)
        let lock1 = NSConditionLock()
        let lock2 = NSConditionLock()
        queue.push(helper: helper, deserializer: des)
        queue.push(helper: helper, deserializer: des)
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
        XCTAssertEqual(helper.calls, 2)
    }

    struct WaitForASecond: CallbackProtocol {
        func call() {
            WallDuration(seconds: 1).sleep()
        }
    }


    func testNonConcurrentOrdering() {
        let queue = SubscriptionQueue(topic: "topic", queueSize: 0, allowConcurrentCallbacks: false)
        let helper = FakeSubHelper()
        let des = MessageDeserializer(helper: helper, m: SerializedMessage(), header: StringStringMap())
        helper.cb = WaitForASecond()
        queue.push(helper: helper, deserializer: des)
        queue.push(helper: helper, deserializer: des)
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
        XCTAssertEqual(helper.calls, 1)
        queue.call()
        XCTAssertEqual(helper.calls, 2)
    }
}

class FakeMessage: Message {
    static var md5sum: String = ""
    static var datatype: String = ""
    static var hasHeader: Bool = false
    static var definition: String = ""
}

protocol CallbackProtocol {
    func call()
}


class FakeSubHelper: SubscriptionCallbackHelper, CustomDebugStringConvertible {
    let hasHeader: Bool = false

    var debugDescription: String { return "Fake \(calls)" }

    public var id: ObjectIdentifier {
        return ObjectIdentifier(self)
    }

    var calls = 0
    var cb : CallbackProtocol?
    var mutex = DispatchQueue(label: "mutex")

    func deserialize(data: [UInt8]) -> Message? {
        return FakeMessage()
    }

    func call(msg: Message, item: SubscriptionQueue.Item) {
        mutex.sync {
            calls += 1
        }
        cb?.call()
    }

}

