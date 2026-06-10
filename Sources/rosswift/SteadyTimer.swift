//
//  SteadyTimer.swift
//  RosTime
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

import RosTime

typealias SteadyTimerCallback = @Sendable (SteadyTimerEvent) async -> Void


/// Manages a steady-timer callback.
///
/// A Timer should always be created through a call to NodeHandle.createSteadyTimer(), or
/// copied from one that was. Once all copies of a specific SteadyTimer go out of scope, the
/// callback associated with that handle will stop being called.


public actor SteadyTimer {
    private static let manager = TimerManager<SteadyTime,SteadyTimerEvent>()
    
    private var started: Bool = false
    private var timerHandle: TimerHandle? = nil
    
    private var period: WallDuration
    let callback: SteadyTimerCallback
    let callbackQueue: AsyncCallbackQueue
    weak var trackedObject: TrackableObject?
    let hasTrackedObject: Bool
    let oneShot: Bool
    
    internal init(period: WallDuration,
                  callback: @escaping SteadyTimerCallback,
                  callbackQueue: AsyncCallbackQueue,
                  trackedObject: TrackableObject?,
                  oneshot: Bool) {
        self.period = period
        self.callback = callback
        self.trackedObject = trackedObject
        self.hasTrackedObject = trackedObject != nil
        self.callbackQueue = callbackQueue
        self.oneShot = oneshot
    }
    
    isolated deinit {
        ROS_DEBUG("Timer deregistering callbacks")
        if started {
            if let handle = timerHandle {
                Task {
                    await SteadyTimer.manager.remove(timerHandle: handle)
                }
            }
        }
    }
    
    func hasStarted() -> Bool {
        return started
    }
    
    func isValid() -> Bool {
        return !period.isZero()
    }
    
    func start() async {
        if !started {
            timerHandle = await SteadyTimer.manager.add(period: period,
                                                        callback: callback,
                                                        callbackQueue: callbackQueue,
                                                        trackedObject: trackedObject,
                                                        oneshot: oneShot)
            started = true
        }
    }
    
    func stop() async {
        if started {
            started = false
            if let handle = timerHandle {
                await SteadyTimer.manager.remove(timerHandle: handle)
                timerHandle = nil
            }
        }
    }
    
    func hasPending() async -> Bool {
        if !isValid() {
            return false
        } else if let handle = timerHandle {
            return await SteadyTimer.manager.hasPending(handle: handle)
        } else {
            return false
        }
    }
    
    func setPeriod(period: WallDuration, reset: Bool = true) async {
        self.period = period
        if let handle = timerHandle {
            await SteadyTimer.manager.setPeriod(handle: handle, period: period, reset: reset)
        }
    }
    
}




public struct SteadyTimerEvent: Event, Sendable {
    public static func createEvent(lastExpected: SteadyTime, lastExpired: SteadyTime, lastReal: SteadyTime, currentExpected: SteadyTime, currentExpired: SteadyTime, currentReal: SteadyTime) -> SteadyTimerEvent {
        return SteadyTimerEvent(lastExpected: lastExpected, lastExpired: lastExpired, lastReal: lastReal, currentExpected: currentExpected, currentExpired: currentExpired, currentReal: currentReal)
        
    }
    
    public let lastExpected: SteadyTime
    public var lastExpired: SteadyTime
    public let lastReal: SteadyTime
    public let currentExpected: SteadyTime
    public var currentExpired: SteadyTime
    public let currentReal: SteadyTime
}

