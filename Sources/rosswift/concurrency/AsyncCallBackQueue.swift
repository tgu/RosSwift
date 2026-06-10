import DequeModule
import Foundation
import NIO
import NIOConcurrencyHelpers
import RosTime


public protocol AsyncCallbackInterface: Actor, Sendable {
    func call() async throws -> CallResult

    /// Provides the opportunity for specifying that a callback is not ready to be called before call() actually takes place.
    var ready: Bool { get }
}

public typealias OwnerType = UUID

enum CallOneResult: Sendable {
    case called
    case tryAgain
    case disabled
    case empty
}

fileprivate struct AsyncCallbackInfo
{
    let callback: AsyncCallbackInterface
    let removal_id: OwnerType
    var markedForRemoval : Bool
}

public actor AsyncCallbackQueue {
    fileprivate var callbacks = Deque<AsyncCallbackInfo>()
    var calling: Int = 0
    var enabled: Bool

    init(enabled: Bool = true) {
        self.enabled = enabled
    }

    public func addCallback(callback: AsyncCallbackInterface, ownerId: OwnerType = UUID()) {
        if enabled {
            let info = AsyncCallbackInfo(callback: callback, removal_id: ownerId, markedForRemoval: false)
            callbacks.append(info)
        }
    }

    public func removeByID(ownerId: OwnerType) {
        for (idx,value) in callbacks.enumerated() {
            if value.removal_id == ownerId {
                callbacks[idx].markedForRemoval = true
            }
        }
    }

    /// Pop a single callback off the front of the queue and invoke it.  If the callback was not ready to be called,
    /// pushes it back onto the queue.  This version includes a timeout which lets you specify the amount of time to wait for
    /// a callback to be available before returning.
    ///
    /// - Parameter timeout: The amount of time to wait for a callback to be available.  If there is already a callback available,
    /// this parameter does nothing.


    func callOne(timeout: WallDuration) async throws -> CallOneResult {
        if !enabled {
            return .disabled
        }

        if !callbacks.isEmpty {
            return try await callOne()
        }

        if !timeout.isZero() {
            try await Task.sleep(for: .seconds(timeout.toSec()))
        }

        return try await callOne()

    }

    //// Invoke all callbacks currently in the queue.  If a callback was not ready to be called, pushes it back onto the queue.

    func callAvailable() async throws {
        try await callAvailable(timeout: WallDuration())
    }

    /// Invoke all callbacks currently in the queue.  If a callback was not ready to be called, pushes it back onto the queue.  This version
    /// includes a timeout which lets you specify the amount of time to wait for a callback to be available before returning.
    ///
    /// - Parameter timeout: The amount of time to wait for at least one callback to be available.  If there is already at least one callback available,
    ///  this parameter does nothing.

    private func checkAvailable(timeout: WallDuration) async -> Deque<AsyncCallbackInfo> {
        if !enabled {
            return []
        }
        if callbacks.isEmpty {
            if !timeout.isZero() {
                try? await Task.sleep(for: .seconds(timeout.toSec()))
            }
            if callbacks.isEmpty {
                return []
            }
            if !enabled {
               return []
            }
        }

        if callbacks.isEmpty {
            return []
        } else {
            let localStorage = callbacks
            callbacks.removeAll()
            calling += localStorage.count
            return localStorage
        }
    }

    func callAvailable(timeout: WallDuration) async throws {
        var localStorage = await checkAvailable(timeout: timeout)
            while !localStorage.isEmpty {
                if try await self.callOne(calls: &localStorage) != .empty {
                    self.calling -= 1
                }
            }
    }

    /// returns whether or not the queue is empty

    var isEmpty: Bool {
        callbacks.isEmpty && calling == 0
    }
    /// Removes all callbacks from the queue.  Does not wait for calls currently in progress to finish.

    public func clear() {
        callbacks.removeAll()
    }

    /// Enable the queue (queue is enabled by default)

    func enable() {
        enabled = true
    }
    /// Disable the queue, meaning any calls to addCallback() will have no effect

    func disable() {
        enabled = false
    }

    /// Returns whether or not this queue is enabled

    var isEnabled: Bool { enabled }

    fileprivate func callOne(calls: inout Deque<AsyncCallbackInfo>) async throws -> CallOneResult {
        var result: CallResult = .invalid

        if let info = calls.popFirst() {
                if !info.markedForRemoval {
                    result = try await info.callback.call()
                }
                if result == .tryAgain && !info.markedForRemoval {
                    calls.append(info)
                } else {
                    result = .success
                }
        }

        if result == .tryAgain {
            return .tryAgain
        }
        return .called
    }

    func callOne() async throws -> CallOneResult {
        var result: CallResult = .invalid

        if let info = callbacks.popFirst() {
                if !info.markedForRemoval {
                    result = try await info.callback.call()
                }
                if result == .tryAgain && !info.markedForRemoval {
                    callbacks.append(info)
                } else {
                    result = .success
                }
        }

        if result == .tryAgain {
            return .tryAgain
        }
        return .called
    }
}
