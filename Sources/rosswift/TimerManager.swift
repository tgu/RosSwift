import Foundation
import RosTime
import Atomics

let gTimerManager = InternalTimerManager()

func getInternalTimerManager() -> InternalTimerManager {
    return gTimerManager
}

typealias InternalTimerManager = TimerManager<SteadyTime, SteadyTimerEvent>

actor TimerManager<T, E: Event> where E.EventTime == T {
    var timers: [TimerHandle: TimerInfo] = [:]
    var newTimer = false
    var waiting = Set<TimerHandle>()
    
    var thread: Task<Void, Error>? = nil
    
    var quit = false
    
    init() {
    }
    
    deinit {
        quit = true
        thread?.cancel()
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
        guard let info = findTimer(handle) else {
            return false
        }
        
        if info.hasTrackedObject && info.trackedObject == nil {
            return false
        }
        
        return info.nextExpected <= T.now || info.waitingCallbacks.load(ordering: .relaxed) != 0
    }
    
    func add(period: T.Duration, callback: @escaping (E) async -> Void, callbackQueue: AsyncCallbackQueue, trackedObject: TrackableObject?, oneshot: Bool) -> TimerHandle {
        let handle = UUID()
        
        let now = T.now
        
        let info = TimerInfo(handle: handle,
                             period: period,
                             callback: callback,
                             callbackQueue: callbackQueue,
                             lastExpected: now,
                             nextExpected: now + period,
                             trackedObject: trackedObject,
                             oneshot: oneshot)
        
        timers[handle] = info
        if thread == nil {
            thread = Task.detached(name: "TimeMagager", priority: .high) {
                try await self.threadFunc()
            }
        }
        
        _ = waiting.insert(handle)
        
        newTimer = true
        
        return handle
    }
    
    func setPeriod(handle: TimerHandle, period: T.Duration, reset: Bool) {
        guard let info = findTimer(handle) else {
            return
        }
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
        newTimer = true
    }
    
    func remove(timerHandle: TimerHandle) async {
        var callbackQueue: AsyncCallbackQueue?
        
        if let info = timers[timerHandle] {
            info.removed = true
            callbackQueue = info.callbackQueue
            timers.removeValue(forKey: timerHandle)
            _ = waiting.remove(timerHandle)
            await callbackQueue?.removeByID(ownerId: info.handle)
        }
        
    }
    
    func schedule(info: borrowing TimerInfo) {
        if info.removed {
            return
        }
        
        info.updateNext(currentTime: T.now)
        _ = waiting.insert(info.handle)
        
        newTimer = true
    }
    
    func threadFunc() async throws {
        var current = T.now
        var sleep_end = T.now
        while !quit {
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
            sleep_end = current + T.Duration(milliseconds: 100)
            
            if !waiting.isEmpty {
                let waitingHandles = waiting.compactMap { findTimer($0) }.sorted(by: { (t1, t2) -> Bool in
                    t1.nextExpected < t2.nextExpected
                })
                
                for ti in waitingHandles {
                    current = T.now
                    if ti.nextExpected <= current {
                        let cb = TimerQueueCallback(parent: self, info: ti, lastExpected: ti.lastExpected, lastReal: ti.lastReal, currentExpected: ti.nextExpected, lastExpired: ti.lastExpired, currentExpired: current)
                        await ti.callbackQueue.addCallback(callback: cb, ownerId: ti.handle)
                        waiting.remove(ti.handle)
                    } else {
                        sleep_end = ti.nextExpected
                        break
                    }
                }
            }
            
            while !newTimer && T.now < sleep_end && !quit {
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
                if !T.isSystemTime {
                    try await Task.sleep(for: .milliseconds(100))
                } else {
                    // On system time we can simply sleep for the rest of the wait time, since anything else requiring processing will
                    // signal the condition variable
                    let remainingTime = max((sleep_end-current).toSec(), 0.001)
                    try await Task.sleep(for: .seconds(remainingTime))
                }
            }
            newTimer = false
        }
    }
    
}

extension TimerManager {
    final class TimerInfo: @unchecked Sendable {
        let handle: TimerHandle
        var period: T.Duration
        
        let callback: (E) async -> Void
        let callbackQueue: AsyncCallbackQueue
        
        var lastCBDuration = WallDuration(nanosec: 0)
        
        var lastExpected: T
        var nextExpected: T
        
        var lastReal: T = T()
        var lastExpired: T = T()
        
        var removed: Bool = false
        weak var trackedObject: TrackableObject?
        let hasTrackedObject: Bool
        let waitingCallbacks = ManagedAtomic(0)
        
        let oneshot: Bool
        
        var totalCalls: UInt32 = 0
        
        init(handle: TimerHandle,
             period: T.Duration,
             callback: @escaping (E) async -> Void,
             callbackQueue: AsyncCallbackQueue,
             lastExpected: T,
             nextExpected: T,
             trackedObject: TrackableObject?,
             oneshot: Bool) {
            
            self.handle = handle
            self.period = period
            self.callback = callback
            self.callbackQueue = callbackQueue
            self.lastExpected = lastExpected
            self.nextExpected = nextExpected
            self.trackedObject = trackedObject
            self.oneshot = oneshot
            self.hasTrackedObject = trackedObject != nil
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
    actor TimerQueueCallback: AsyncCallbackInterface {
        let parent: TimerManager<T,E>
        var info: TimerManager<T,E>.TimerInfo
        let lastExpected: T
        let lastReal: T
        let currentExpected: T
        let lastExpired: T
        let currentExpired: T
        var called: Bool
        
        init(parent: TimerManager<T,E>,
             info: TimerManager<T,E>.TimerInfo,
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
            
            info.waitingCallbacks.wrappingIncrement(ordering: .relaxed)
        }
        
        isolated deinit {
            info.waitingCallbacks.wrappingDecrement(ordering: .relaxed)
        }
        
        func call() async -> CallResult {
            info.totalCalls += 1
            called = true
            
            if info.hasTrackedObject {
                if info.trackedObject == nil {
                    return .invalid
                }
            }
            
            let event = E.createEvent(lastExpected: lastExpected,
                                      lastExpired: lastExpired,
                                      lastReal: lastReal,
                                      currentExpected: currentExpected,
                                      currentExpired: currentExpired,
                                      currentReal: T.now)
            
            let cbStart = SteadyTime.now
            await info.callback(event)
            let cbEnd = SteadyTime.now
            info.lastCBDuration = cbEnd - cbStart
            info.lastReal = event.currentReal
            info.lastExpired = event.currentExpired
            
            await parent.schedule(info: info)
            
            return .success
            
        }
        
        let ready = true
    }
}
