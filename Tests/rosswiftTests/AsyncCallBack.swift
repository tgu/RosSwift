

import Testing
@testable import RosSwift
@testable import RosTime
import Synchronization

final actor AsyncCountingCallback: AsyncCallbackInterface, Sendable {
    let ready = true
    var count = 0

    func call() -> CallResult {
        count += 1
        return .success
    }
}


struct AsyncCallbackQueueTests {

    @Test func singleCallback() async throws {
        let cb = AsyncCountingCallback()
        let queue = AsyncCallbackQueue()

        let id = OwnerType()

        await queue.addCallback(callback: cb, ownerId: id)
        _ = try await queue.callOne()
        #expect(await cb.count == 1)


        await queue.addCallback(callback: cb, ownerId: id)
        try await queue.callAvailable()
        #expect(await cb.count == 2)

        _ = try await queue.callOne()
        #expect(await cb.count == 2)

        try await queue.callAvailable()
        #expect(await cb.count == 2)

    }

    @Test func multipleCallbacksCallAvailable() async throws {
        let cb = AsyncCountingCallback()
        let queue = AsyncCallbackQueue()
        for _ in 0..<1000 {
            await queue.addCallback(callback: cb)
        }
        
        try await queue.callAvailable()
        #expect(await cb.count == 1000)
    }

    @Test func multipleCallbacksCallOne() async throws {
        let cb = AsyncCountingCallback()
        let queue = AsyncCallbackQueue()
        for _ in 0..<1000 {
            await queue.addCallback(callback: cb)
        }

        for i in 1...1000 {
            _ = try await queue.callOne()
            #expect(await cb.count == i)
        }
    }

    @Test func remove() async throws {
        let cb1 = AsyncCountingCallback()
        let cb2 = AsyncCountingCallback()
        let queue = AsyncCallbackQueue()
        let id1 = OwnerType()
        let id2 = OwnerType()
        await queue.addCallback(callback: cb1, ownerId: id1)
        await queue.addCallback(callback: cb2, ownerId: id2)
        await queue.removeByID(ownerId: id1)
        try await queue.callAvailable()
        #expect(await cb1.count == 0)
        #expect(await  cb2.count == 1)
    }

    actor SelfRemovingCallback: AsyncCallbackInterface {
        var count: Int
        let queue: AsyncCallbackQueue
        let id: OwnerType

        init(queue: AsyncCallbackQueue, id: OwnerType) {
            self.queue = queue
            self.id = id
            self.count = 0
        }

        func call() async -> CallResult {
            count += 1
            await queue.removeByID(ownerId: id)
            return .success
        }

        let ready = true
    }

    @Test func removeSelf() async throws {
        let queue = AsyncCallbackQueue()
        let id = OwnerType()
        let cb1 = SelfRemovingCallback(queue: queue, id: id)
        let cb2 = AsyncCountingCallback()
        await queue.addCallback(callback: cb1, ownerId: id)
        await queue.addCallback(callback: cb2, ownerId: id)
        await queue.addCallback(callback: cb2, ownerId: id)

        _ = try await queue.callOne()

        await queue.addCallback(callback: cb2, ownerId: id)

        try await queue.callAvailable()

        #expect(await cb1.count == 1)
        #expect(await cb2.count == 1)

    }

    actor RecursiveCallback: AsyncCallbackInterface {
        var count: Int
        let queue: AsyncCallbackQueue
        let useAvailable: Bool

        init(queue: AsyncCallbackQueue, useAvailable: Bool) {
            self.queue = queue
            self.useAvailable = useAvailable
            self.count = 0
        }

        func call() async throws -> CallResult {
            count += 1
            if count < 3 {
                if useAvailable {
                    try await queue.callAvailable()
                } else {
                _ = try await queue.callOne()
                }
            }
            return .success
        }

        let ready = true
    }

    @Test func recursive1() async throws {
        let queue = AsyncCallbackQueue()
        let cb = RecursiveCallback(queue: queue, useAvailable: true)
        let id = OwnerType()
        await queue.addCallback(callback: cb, ownerId: id)
        await queue.addCallback(callback: cb, ownerId: id)
        await queue.addCallback(callback: cb, ownerId: id)
        try await queue.callAvailable()
        #expect(await cb.count == 3)
    }

    @Test func recursive2() async throws {
        let queue = AsyncCallbackQueue()
        let cb = RecursiveCallback(queue: queue, useAvailable: false)
        let id = OwnerType()
        await queue.addCallback(callback: cb, ownerId: id)
        await queue.addCallback(callback: cb, ownerId: id)
        await queue.addCallback(callback: cb, ownerId: id)
        _ = try await queue.callOne()
        #expect(await cb.count == 3)

    }

    @Test func recursive3() async throws {
        let queue = AsyncCallbackQueue()
        let cb = RecursiveCallback(queue: queue, useAvailable: false)
        let id = OwnerType()
        await queue.addCallback(callback: cb, ownerId: id)
        await queue.addCallback(callback: cb, ownerId: id)
        await queue.addCallback(callback: cb, ownerId: id)
        _ = try await queue.callOne()
        #expect(await cb.count == 3)

    }

    @Test func recursive4() async throws {
        let queue = AsyncCallbackQueue()
        let cb = RecursiveCallback(queue: queue, useAvailable: true)
        let id = OwnerType()
        await queue.addCallback(callback: cb, ownerId: id)
        await queue.addCallback(callback: cb, ownerId: id)
        await queue.addCallback(callback: cb, ownerId: id)
        _ = try await queue.callOne()
        #expect(await cb.count == 3)

    }

}
