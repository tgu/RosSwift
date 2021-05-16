//
//  WallTimer.swift
//  RosTime
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

import RosTime


typealias WallTimerCallback = (WallTimerEvent) -> Void


/// Manages a wall-timer callback.
///
/// A Timer should always be created through a call to NodeHandle.createWallTimer(), or
/// copied from one that was. Once all copies of a specific SteadyTimer go out of scope, the
/// callback associated with that handle will stop being called.


public final class WallTimer {
    private static let manager = TimerManager<WallTime,WallDuration,WallTimerEvent>()

    private var started: Bool = false
    private var timerHandle: TimerHandle = .none

    private var period: WallDuration
    let callback: WallTimerCallback
    let callbackQueue: CallbackQueueInterface
    let trackedObject: AnyObject?
    let hasTrackedObject: Bool
    let oneShot: Bool

    internal init(period: WallDuration,
                  callback: @escaping WallTimerCallback,
                  callbackQueue: CallbackQueueInterface,
                  trackedObject: AnyObject?,
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
        stop()
    }

    func hasStarted() -> Bool {
        return started
    }

    func isValid() -> Bool {
        return !period.isZero()
    }

    func start() {
        if !started {
            timerHandle = WallTimer.manager.add(period: period,
                                                  callback: callback,
                                                  callbackQueue: callbackQueue,
                                                  trackedObject: trackedObject,
                                                  oneshot: oneShot)
            started = true
        }
    }

    func stop() {
        if started {
            started = false
            WallTimer.manager.remove(timerHandle: timerHandle)
            timerHandle = .none
        }
    }

    func hasPending() -> Bool {
        if !isValid() || timerHandle.isNone {
            return false
        }

        return WallTimer.manager.hasPending(handle: timerHandle)
    }

    func setPeriod(period: WallDuration, reset: Bool = true) {
        self.period = period
        WallTimer.manager.setPeriod(handle: timerHandle, period: period, reset: reset)
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


