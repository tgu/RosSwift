//
//  CallBackQueue.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-11-20.
//

import Deque
import Foundation
import NIO
import NIOConcurrencyHelpers
import RosTime

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

public typealias OwnerType = Int

public protocol CallbackQueueInterface {

    /// Add a callback, with an optional owner id.  The owner id can be used to
    /// remove a set of callbacks from this queue.

    func addCallback(callback: CallbackInterface, ownerId: OwnerType)

    /// Remove all callbacks associated with an owner id

    func removeByID(ownerId: OwnerType)
}

enum CallOneResult {
    case called
    case tryAgain
    case disabled
    case empty
}

fileprivate struct IDInfo {
    let id: OwnerType
    let callingMutex = NSRecursiveLock()
}

fileprivate struct CallbackInfo
{
    let callback: CallbackInterface?
    let removal_id: OwnerType
    var markedForRemoval : Bool
}

fileprivate final class ThreadLocalStorage {
    static let noThread = OwnerType.max
    var callingInThisThread = ThreadLocalStorage.noThread
    var callbacksIndex: Deque<CallbackInfo>.Index
    var callbacks = Deque<CallbackInfo>()
    init(index: Deque<CallbackInfo>.Index) {
        callbacksIndex = index
    }
}

internal final class CallbackQueue: CallbackQueueInterface {
    private var callbacks = Deque<CallbackInfo>()
    private var calling: Int = 0
    private let condition = NSCondition()
    private let idInfoMutex = DispatchQueue(label: "id_info_mutex")
    private var idInfo = [OwnerType: IDInfo]()

    fileprivate var tls: ThreadSpecificVariable<ThreadLocalStorage>

    var enabled: Bool

    init(enabled: Bool = true) {
        self.enabled = enabled
        self.tls = ThreadSpecificVariable()
    }

    func addCallback(callback: CallbackInterface, ownerId: OwnerType = 0) {
        let info = CallbackInfo(callback: callback, removal_id: ownerId, markedForRemoval: false)
        idInfoMutex.sync {
            if idInfo[ownerId] == nil {
                idInfo[ownerId] = IDInfo(id: ownerId)
            }
        }
        condition.sync {
            if !enabled {
                return
            }
            callbacks.append(info)
        }

        condition.signal()
    }

    func removeByID(ownerId: OwnerType) {
        setupTLS()
        guard let info = idInfoMutex.sync(execute: { idInfo[ownerId] }) else {
            return
        }

        // If we're being called from within a callback from our queue, we must unlock the shared lock we already own
        // here so that we can take a unique lock.  We'll re-lock it later.

        if let localStorage = tls.currentValue, localStorage.callingInThisThread == info.id {
            info.callingMutex.sync {
                condition.sync {
                    for index in (localStorage.callbacksIndex..<callbacks.endIndex).reversed() {
                        if callbacks[index].removal_id == ownerId {
                            callbacks.remove(at: index)
                        }
                    }
                }
            }
        }

        // If we're being called from within a callback, we need to remove the callbacks that match the id that have already been
        // popped off the queue

        for (idx,value) in callbacks.enumerated() {
            if value.removal_id == ownerId {
                callbacks[idx].markedForRemoval = true
            }
        }

        idInfoMutex.sync {
            _ = idInfo.removeValue(forKey: ownerId)
        }
    }


    /// Pop a single callback off the front of the queue and invoke it.  If the callback was not ready to be called,
    /// pushes it back onto the queue.

    func callOne() -> CallOneResult {
        return callOne(timeout: WallDuration())
    }

    /// Pop a single callback off the front of the queue and invoke it.  If the callback was not ready to be called,
    /// pushes it back onto the queue.  This version includes a timeout which lets you specify the amount of time to wait for
    /// a callback to be available before returning.
    ///
    /// - Parameter timeout: The amount of time to wait for a callback to be available.  If there is already a callback available,
    /// this parameter does nothing.


    func callOne(timeout: WallDuration) -> CallOneResult {
        setupTLS()
        var cbInfo: CallbackInfo?
        var res: CallOneResult?
        condition.sync {
            if !enabled {
                res = .disabled
                return
            }
            if callbacks.isEmpty {
                if !timeout.isZero() {
                    condition.wait(until: Date(timeIntervalSinceNow: timeout.toSec()))
                }
                if callbacks.isEmpty {
                    res = .empty
                    return
                }
                if !enabled {
                    res = .disabled
                   return
                }
            }
            for (idx,item) in callbacks.enumerated().reversed() {
                if item.markedForRemoval {
                    callbacks.remove(at: idx)
                    continue
                }
                if let cb = item.callback, cb.ready() {
                    cbInfo = item
                    callbacks.remove(at: idx)
                    break
                }
            }
            if cbInfo?.callback == nil {
                res = .tryAgain
                return
            }
            calling += 1
        }

        if res != nil {
            return res!
        }

        guard var localStorage = tls.currentValue else {
            fatalError("no local storage")
        }
        guard let info = cbInfo else {
            fatalError("internal error")
        }
        let wasEmpty = localStorage.callbacks.isEmpty
        localStorage.callbacks.append(info)
        if wasEmpty {
            localStorage.callbacksIndex = localStorage.callbacks.startIndex
        }
        let result = callOneCB(tls: &localStorage)
        if result != .empty {
            condition.sync {
                calling -= 1
            }
        }
        return result
    }

    //// Invoke all callbacks currently in the queue.  If a callback was not ready to be called, pushes it back onto the queue.

    func callAvailable() {
        callAvailable(timeout: WallDuration())
    }

    /// Invoke all callbacks currently in the queue.  If a callback was not ready to be called, pushes it back onto the queue.  This version
    /// includes a timeout which lets you specify the amount of time to wait for a callback to be available before returning.
    ///
    /// - Parameter timeout: The amount of time to wait for at least one callback to be available.  If there is already at least one callback available,
    ///  this parameter does nothing.

    func callAvailable(timeout: WallDuration) {
        setupTLS()
        var empty = true
        condition.sync {
            if !enabled {
                return
            }
            if callbacks.isEmpty {
                if !timeout.isZero() {
                    condition.wait(until: Date(timeIntervalSinceNow: timeout.toSec()))
                }
                if callbacks.isEmpty {
                    return
                }
                if !enabled {
                   return
                }
            }
            guard let localStorage = tls.currentValue else {
                fatalError("no local storage")
            }
            let wasEmpty = localStorage.callbacks.isEmpty
            localStorage.callbacks.append(contentsOf: callbacks)
            callbacks.removeAll()
            calling += localStorage.callbacks.count
            if wasEmpty {
                localStorage.callbacksIndex = localStorage.callbacks.startIndex
            }
            empty = false
        }
        if empty {
            return
        }

        var called = 0

        guard var localStorage = tls.currentValue else {
            fatalError("no local storage")
        }

        while !localStorage.callbacks.isEmpty {
            if callOneCB(tls: &localStorage) != .empty {
                called += 1
            }
        }
        condition.sync {
            calling -= called
        }
    }

    /// returns whether or not the queue is empty

    var isEmpty: Bool {
        return condition.sync {
            callbacks.isEmpty && calling == 0
        }
    }
    /// Removes all callbacks from the queue.  Does not wait for calls currently in progress to finish.

    func clear() {
        condition.sync {
            callbacks.removeAll()
        }
    }

    /// Enable the queue (queue is enabled by default)

    func enable() {
        condition.sync {
            enabled = true
        }
        condition.broadcast()
    }
    /// Disable the queue, meaning any calls to addCallback() will have no effect

    func disable() {
        condition.sync {
            enabled = false
        }
        condition.broadcast()
    }

    /// Returns whether or not this queue is enabled

    var isEnabled: Bool {
        return condition.sync { enabled }
    }

    fileprivate func setupTLS() {
        if tls.currentValue == nil {
            tls.currentValue = ThreadLocalStorage(index: self.callbacks.endIndex)
        }
    }

    fileprivate func callOneCB(tls: inout ThreadLocalStorage) -> CallOneResult {
        // Check for a recursive call.  If recursive, increment the current iterator.  Otherwise
        // set the iterator it the beginning of the thread-local callbacks
        if tls.callingInThisThread == ThreadLocalStorage.noThread {
            tls.callbacksIndex = tls.callbacks.startIndex
        }
        if tls.callbacksIndex == tls.callbacks.endIndex {
            return .empty
        }
        precondition(!tls.callbacks.isEmpty)
        precondition(tls.callbacksIndex != tls.callbacks.endIndex)
        let index = tls.callbacksIndex
        let info = tls.callbacks[index]
        var result = CallResult.invalid
        if let idInfo = getIDInfo(id: info.removal_id) {
            idInfo.callingMutex.sync {
                tls.callingInThisThread = idInfo.id
                if info.markedForRemoval {
                    tls.callbacks.remove(at: index)
                } else {
                    tls.callbacks.remove(at: index)
                    result = info.callback?.call() ?? .invalid
                }
                if result == .tryAgain && !callbacks[index].markedForRemoval {
                    condition.sync {
                        callbacks.append(info)
                    }
                } else {
                    result = .success
                }
            }
        } else {
            tls.callbacks.remove(at: index)
            result = .success
        }

        if result == .tryAgain {
            return .tryAgain
        }
        return .called
    }

    fileprivate func getIDInfo(id: OwnerType) -> IDInfo? {
        return idInfoMutex.sync { idInfo[id] }
    }


}
