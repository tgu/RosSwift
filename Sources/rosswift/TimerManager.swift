import Foundation
import RosTime
import NIOConcurrencyHelpers

let gTimerManager = InternalTimerManager()

func getInternalTimerManager() -> InternalTimerManager {
    return gTimerManager
}

typealias InternalTimerManager = TimerManager<SteadyTime, WallDuration, SteadyTimerEvent>

func initInternalTimerManager() {
    ROS_ERROR("initInternalTimerManager not implemented")
}

final class TimerManager<T, D: BasicDurationBase, E: Event> where E.EventTime == T {
    var timers: [TimerHandle: TimerInfo] = [:]
    let timersMutex = DispatchQueue(label: "TimerManager", attributes: .concurrent)
    let timersCond = NSCondition()
    var newTimer = Atomic<Bool>(value: false)
    let waitingMutex = DispatchQueue(label: "waitingMutex")
    var waiting = Set<TimerHandle>()

    var threadStarted = false
    var thread: Thread? = nil
    let threadGroup = DispatchGroup()

    var quit = false

    init() {
    }

    deinit {
        quit = true
        timersMutex.sync {
            timersCond.broadcast()
        }
        if threadStarted {
            threadGroup.wait()
        }
    }

    func waitingCompare(lhs: TimerHandle, rhs: TimerHandle) -> Bool {
        let infol = findTimer(lhs)
        let infor = findTimer(rhs)

        if let infol = infol, let infor = infor {
            return infol.nextExpected < infor.nextExpected
        }

        return infor != nil
    }

    func findTimer(_ handle: TimerHandle) -> TimerInfo? {
        return timers[handle]
    }

    func hasPending(handle: TimerHandle) -> Bool {
        return timersMutex.sync {
            guard let info = findTimer(handle) else {
                return false
            }

            if info.hasTrackedObject && info.trackedObject == nil {
                return false
            }

            return info.nextExpected <= T.now || info.waitingCallbacks.load() != 0
        }
    }

    func add(period: D, callback: @escaping (E) -> Void, callbackQueue: CallbackQueueInterface, trackedObject: AnyObject?, oneshot: Bool) -> TimerHandle {
        let handle = TimerHandle.getNextHandler()

        let now = T.now

        let info = TimerInfo(handle: handle,
                             period: period,
                             callback: callback,
                             callbackQueue: callbackQueue,
                             lastExpected: now,
                             nextExpected: now + period,
                             trackedObject: trackedObject,
                             oneshot: oneshot)

        timersMutex.sync(flags: .barrier) {
            timers[handle] = info
            if !threadStarted {
                thread = Thread(block: threadFunc)
                thread?.start()
                threadStarted = true
            }

            waitingMutex.sync {
                _ = waiting.insert(handle)
            }
        }

        newTimer.store(true)
        timersCond.broadcast()

        return handle
    }

    func setPeriod(handle: TimerHandle, period: D, reset: Bool) {
        timersMutex.sync(flags: .barrier) {
            guard let info = findTimer(handle) else {
                return
            }
            //            waitingMutex.sync {
            if reset {
                info.nextExpected = T.now + period

                // else if some time has elapsed since last cb (called outside of cb)
            } else if (T.now - info.lastReal).nanoseconds < info.period.nanoseconds {
                // if elapsed time is greater than the new period
                // do the callback now
                if (T.now - info.lastReal).nanoseconds > period.nanoseconds {
                    info.nextExpected = T.now
                } else {
                    // else, account for elapsed time by using last_real+period
                    info.nextExpected = info.lastReal + period
                }
            }

            // Else if called in a callback, last_real has not been updated yet => (now - last_real) > period
            // In this case, let next_expected be updated only in updateNext

            info.period = period
            //            }
        }
        newTimer.store(true)
        timersCond.broadcast()
    }

    func remove(timerHandle: TimerHandle) {
        var callbackQueue: CallbackQueueInterface?
        var removeId: Int = 0

        timersMutex.sync(flags: .barrier) {
            if let info = timers[timerHandle] {
                info.removed = true
                callbackQueue = info.callbackQueue
                removeId = info.handle.hashValue
                timers.removeValue(forKey: timerHandle)

                waitingMutex.sync {
                    _ = waiting.remove(timerHandle)
                }
            }
        }

        callbackQueue?.removeByID(ownerId: removeId)
    }

    func schedule(info: inout TimerInfo) {
        timersMutex.sync(flags: .barrier) {
            if info.removed {
                return
            }

            info.updateNext(currentTime: T.now)
            waitingMutex.sync {
                _ = waiting.insert(info.handle)
            }
        }

        newTimer.store(true)
        timersCond.broadcast()
    }

    func threadFunc() {
        var current = T.now
        var sleep_end = T.now
        while !quit {
            timersMutex.sync(flags: .barrier) {
                // detect time jumping backwards
                if T.now < current {
                    ROS_DEBUG("Time jumped backward, resetting time")
                    current = T.now
                    for info in timers.values.filter({ $0.lastExpected > current }) {
                        // Timer may have been added after the time jump, so also check if time has jumped past its last call time
                        info.lastExpected = current
                        info.nextExpected = current + info.period
                    }
                }

                current = T.now
                waitingMutex.sync {
                    sleep_end = current + D(milliseconds: 100)

                    if !waiting.isEmpty {
                        let waitingHandles = waiting.compactMap { findTimer($0) }.sorted(by: { (t1, t2) -> Bool in
                            t1.nextExpected < t2.nextExpected
                        })

                        for var ti in waitingHandles {
                            current = T.now
                            if ti.nextExpected <= current {
                                let cb = TimerQueueCallback(parent: self, info: &ti, lastExpected: ti.lastExpected, lastReal: ti.lastReal, currentExpected: ti.nextExpected, lastExpired: ti.lastExpired, currentExpired: current)
                                ti.callbackQueue.addCallback(callback: cb, ownerId: ti.handle.hashValue)
                                waiting.remove(ti.handle)
                            } else {
                                sleep_end = ti.nextExpected
                                break
                            }
                        }
                    }
                }
            }
            
            while !newTimer.load() && T.now < sleep_end && !quit {
                // detect backwards jump in time
                if T.now < current {
                    ROS_DEBUG("Time jumped backwards, breaking out of sleep")
                    break
                }
                current = T.now
                if current >= sleep_end {
                    break
                }
                // If we're on simulation time we need to check now() against sleep_end more often than on system time,
                // since simulation time may be running faster than real time.
                if !T.isSystemTime() {
                    timersCond.wait(until: Date(timeIntervalSinceNow: 0.1))
                } else {
                    // On system time we can simply sleep for the rest of the wait time, since anything else requiring processing will
                    // signal the condition variable
                    let remainingTime = max((sleep_end-current).toSec(), 0.001)
                    timersCond.wait(until: Date(timeIntervalSinceNow: remainingTime))
                }
            }
            newTimer.store(false)
        }
    }

}

extension TimerManager {
    final class TimerInfo {
        let handle: TimerHandle
        var period: D

        let callback: (E) -> Void
        let callbackQueue: CallbackQueueInterface

        var lastCBDuration = WallDuration(nanosec: 0)

        var lastExpected: T
        var nextExpected: T

        var lastReal: T = T()
        var lastExpired: T = T()

        var removed: Bool = false
        let trackedObject: AnyObject?
        var hasTrackedObject: Bool { return trackedObject != nil }
        var waitingCallbacks = Atomic<UInt32>(value: 0)

        let oneshot: Bool

        var totalCalls: UInt32 = 0

        init(handle: TimerHandle,
             period: D,
             callback: @escaping (E) -> Void,
             callbackQueue: CallbackQueueInterface,
             lastExpected: T,
             nextExpected: T,
             trackedObject: AnyObject?,
             oneshot: Bool) {

            self.handle = handle
            self.period = period
            self.callback = callback
            self.callbackQueue = callbackQueue
            self.lastExpected = lastExpected
            self.nextExpected = nextExpected
            self.trackedObject = trackedObject
            self.oneshot = oneshot
        }

        func updateNext(currentTime: T) {
            if oneshot {
                nextExpected = T.distantFuture()

            } else {
                // Protect against someone having called setPeriod()
                // If the next expected time is already past the current time
                // don't update it

                if nextExpected <= currentTime {
                    lastExpected = nextExpected
                    nextExpected += period
                }

                // detect time jumping forward, as well as callbacks that are too slow
                if nextExpected + period < currentTime {
                    ROS_DEBUG("Time jumped forward by [\((currentTime-nextExpected).toSec())] for timer of period [\(period.toSec())], resetting timer (current=\(currentTime.toSec()) next_expected=\(nextExpected.toSec())")
                    nextExpected = currentTime
                }
            }
        }

    }
}

extension TimerManager {
    class TimerQueueCallback: CallbackInterface {
        let parent: TimerManager<T,D,E>
        var info: TimerManager<T,D,E>.TimerInfo
        let lastExpected: T
        let lastReal: T
        let currentExpected: T
        let lastExpired: T
        let currentExpired: T
        var called: Bool

        init(parent: TimerManager<T,D,E>,
             info: inout TimerManager<T,D,E>.TimerInfo,
             lastExpected: T,
             lastReal: T,
             currentExpected: T,
             lastExpired: T,
             currentExpired: T) {

            self.parent = parent
            self.info = info
            self.lastExpected = lastExpected
            self.lastReal = lastReal
            self.currentExpected = currentExpected
            self.lastExpired = lastExpired
            self.called = false
            self.currentExpired = currentExpired

            _ = info.waitingCallbacks.add(1)
        }

        deinit {
            _ = info.waitingCallbacks.sub(1)
        }

        func call() -> CallResult {
            info.totalCalls += 1
            called = true

            let event = E.createEvent(lastExpected: lastExpected,
                                      lastExpired: lastExpired,
                                      lastReal: lastReal,
                                      currentExpected: currentExpected,
                                      currentExpired: currentExpired,
                                      currentReal: T.now)

            let cbStart = SteadyTime.now
            info.callback(event)
            let cbEnd = SteadyTime.now
            info.lastCBDuration = cbEnd - cbStart
            info.lastReal = event.currentReal
            info.lastExpired = event.currentExpired
            parent.schedule(info: &info)

            return .success
            
        }

        func ready() -> Bool {
            return true
        }
    }
}
