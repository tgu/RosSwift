

import XCTest
@testable import RosSwift
@testable import RosTime

class CountingCallback: CallbackInterface {
    let mutex = DispatchQueue(label: "mutex")
    var count = 0

    func call() -> CallResult {
        mutex.sync {
            count += 1
        }
        return .success
    }

    func ready() -> Bool {
        return true
    }
}


class CallbackQueueTests: XCTestCase {

    static var allTests = [
        ("testSingleCallback",testSingleCallback)
    ]


    override func setUp() {
    }

    override func tearDown() {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }

    func testSingleCallback() {
        let cb = CountingCallback()
        let queue = CallbackQueue()

        queue.addCallback(callback: cb, ownerId: 1)
        _ = queue.callOne()
        XCTAssertEqual(cb.count, 1)

        queue.addCallback(callback: cb, ownerId: 1)
        queue.callAvailable()
        XCTAssertEqual(cb.count, 2)

        _ = queue.callOne()
        XCTAssertEqual(cb.count, 2)

        queue.callAvailable()
        XCTAssertEqual(cb.count, 2)
    }

    func testMultipleCallbacksCallAvailable() {
        let cb = CountingCallback()
        let queue = CallbackQueue()
        for _ in 0..<1000 {
            queue.addCallback(callback: cb, ownerId: 1)
        }
        queue.callAvailable()
        XCTAssertEqual(cb.count, 1000)
    }

    func testMultipleCallbacksCallOne() {
        let cb = CountingCallback()
        let queue = CallbackQueue()
        for _ in 0..<1000 {
            queue.addCallback(callback: cb, ownerId: 1)
        }

        for i in 1...1000 {
            _ = queue.callOne()
            XCTAssertEqual(cb.count, i)
        }
    }

    func testRemove() {
        let cb1 = CountingCallback()
        let cb2 = CountingCallback()
        let queue = CallbackQueue()
        queue.addCallback(callback: cb1, ownerId: 1)
        queue.addCallback(callback: cb2, ownerId: 2)
        queue.removeByID(ownerId: 1)
        queue.callAvailable()
        XCTAssertEqual(cb1.count, 0)
        XCTAssertEqual(cb2.count, 1)
    }

    class SelfRemovingCallback: CallbackInterface {
        var count: Int
        let queue: CallbackQueue
        let id: OwnerType

        init(queue: CallbackQueue, id: OwnerType) {
            self.queue = queue
            self.id = id
            self.count = 0
        }

        func call() -> CallResult {
            count += 1
            queue.removeByID(ownerId: id)
            return .success
        }

        func ready() -> Bool {
            return true
        }
    }

    func testRemoveSelf() {
        let queue = CallbackQueue()
        let cb1 = SelfRemovingCallback(queue: queue, id: 1)
        let cb2 = CountingCallback()
        queue.addCallback(callback: cb1, ownerId: 1)
        queue.addCallback(callback: cb2, ownerId: 1)
        queue.addCallback(callback: cb2, ownerId: 1)

        _ = queue.callOne()

        queue.addCallback(callback: cb2, ownerId: 1)

        queue.callAvailable()

        XCTAssertEqual(cb1.count, 1)
        XCTAssertEqual(cb2.count, 1)

    }

    class RecursiveCallback: CallbackInterface {
        var count: Int
        let queue: CallbackQueue
        let useAvailable: Bool

        init(queue: CallbackQueue, useAvailable: Bool) {
            self.queue = queue
            self.useAvailable = useAvailable
            self.count = 0
        }

        func call() -> CallResult {
            count += 1
            if count < 3 {
                if useAvailable {
                    queue.callAvailable()
                } else {
                _ = queue.callOne()
                }
            }
            return .success
        }

        func ready() -> Bool {
            return true
        }
    }

    func testRecursive1() {
        let queue = CallbackQueue()
        let cb = RecursiveCallback(queue: queue, useAvailable: true)
        queue.addCallback(callback: cb, ownerId: 1)
        queue.addCallback(callback: cb, ownerId: 1)
        queue.addCallback(callback: cb, ownerId: 1)
        queue.callAvailable()
        XCTAssertEqual(cb.count, 3)

    }

    func testRecursive2() {
        let queue = CallbackQueue()
        let cb = RecursiveCallback(queue: queue, useAvailable: false)
        queue.addCallback(callback: cb, ownerId: 1)
        queue.addCallback(callback: cb, ownerId: 1)
        queue.addCallback(callback: cb, ownerId: 1)
        _ = queue.callOne()
        XCTAssertEqual(cb.count, 3)

    }

    func testRecursive3() {
        let queue = CallbackQueue()
        let cb = RecursiveCallback(queue: queue, useAvailable: false)
        queue.addCallback(callback: cb, ownerId: 1)
        queue.addCallback(callback: cb, ownerId: 1)
        queue.addCallback(callback: cb, ownerId: 1)
        _ = queue.callOne()
        XCTAssertEqual(cb.count, 3)

    }

    func testRecursive4() {
        let queue = CallbackQueue()
        let cb = RecursiveCallback(queue: queue, useAvailable: true)
        queue.addCallback(callback: cb, ownerId: 1)
        queue.addCallback(callback: cb, ownerId: 1)
        queue.addCallback(callback: cb, ownerId: 1)
        _ = queue.callOne()
        XCTAssertEqual(cb.count, 3)

    }

    func callAvailableThread(queue: CallbackQueue, done: inout Bool) {
        while !done {
            _ = queue.callOne(timeout: WallDuration(seconds: 0.1))
        }
    }

    func runThreadedTest(cb: CallbackInterface, threadFunc: @escaping (CallbackQueue,inout Bool) -> Void) -> Int {
        let queue = CallbackQueue()
        var done = false
        var threads = [Thread]()
        for _ in 0..<10 {
            let thread = Thread { threadFunc(queue,&done) }
            threads.append(thread)
            thread.start()
        }
        let start = WallTime.now
        var i = 0
        while (WallTime.now - start).nanoseconds < Duration(seconds: 5).nanoseconds {
            queue.addCallback(callback: cb)
            i += 1
        }

        while !queue.isEmpty {
            WallDuration(milliseconds: 10).sleep()
        }

        done = true

        return i
    }

    func testThreadedCallAvailable() {
        let cb = CountingCallback()
        let i = runThreadedTest(cb: cb, threadFunc: callAvailableThread )
        XCTAssertEqual(cb.count, i)
        print(i)
    }

    func callOneThread(queue: CallbackQueue, done: inout Bool) {
        while !done {
            _ = queue.callOne(timeout: WallDuration(milliseconds: 100))
        }
    }

    func testThreadedCallOne() {
        let cb = CountingCallback()
        let i = runThreadedTest(cb: cb, threadFunc: callOneThread )
        print(i)
        XCTAssertEqual(cb.count, i)
    }


}
