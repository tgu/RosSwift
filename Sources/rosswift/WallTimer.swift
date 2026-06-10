//
//  WallTimer.swift
//  RosTime
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

import RosTime


typealias WallTimerCallback = @Sendable (WallTimerEvent) -> Void


/// Manages a wall-timer callback.
///
/// A Timer should always be created through a call to NodeHandle.createWallTimer(), or
/// copied from one that was. Once all copies of a specific SteadyTimer go out of scope, the
/// callback associated with that handle will stop being called.


public actor WallTimer {
    private static let manager = TimerManager<WallTime,WallTimerEvent>()
    
    private var started: Bool = false
    private var timerHandle: TimerHandle? = nil
    
    private var period: WallDuration
    let callback: WallTimerCallback
    let callbackQueue: AsyncCallbackQueue
    weak var trackedObject: TrackableObject?
    let hasTrackedObject: Bool
    let oneShot: Bool
    
    internal init(period: WallDuration,
                  callback: @escaping WallTimerCallback,
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
    
    deinit {
        ROS_DEBUG("Timer deregistering callbacks")
        fatalError()
        //stop()
    }
    
    func hasStarted() -> Bool {
        return started
    }
    
    func isValid() -> Bool {
        return !period.isZero()
    }
    
    func start() async {
        if !started {
            timerHandle = await WallTimer.manager.add(period: period,
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
                await WallTimer.manager.remove(timerHandle: handle)
                timerHandle = nil
            }
        }
    }
    
    func hasPending() async -> Bool {
        if !isValid() {
            return false
        } else if let handle = timerHandle {
            return await WallTimer.manager.hasPending(handle: handle)
        } else {
            return false
        }
    }
    
    func setPeriod(period: WallDuration, reset: Bool = true) async {
        self.period = period
        if let handle = timerHandle {
            await WallTimer.manager.setPeriod(handle: handle, period: period, reset: reset)
        }
    }
    
}

public struct WallTimerEvent: Event {
    public typealias EventTime = WallTime
    
    public static func createEvent(lastExpected: WallTime, lastExpired: WallTime, lastReal: WallTime, currentExpected: WallTime, currentExpired: WallTime, currentReal: WallTime) -> WallTimerEvent {
        return WallTimerEvent(lastExpected: lastExpected, lastExpired: lastExpired, lastReal: lastReal, currentExpected: currentExpected, currentExpired: currentExpired, currentReal: currentReal)
    }
    
    public let lastExpected: WallTime
    public var lastExpired: WallTime
    public let lastReal: WallTime
    public let currentExpected: WallTime
    public var currentExpired: WallTime
    public let currentReal: WallTime
}


