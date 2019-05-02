//
//  SteadyTimer.swift
//  RosTime
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

import Foundation
import RosTime

typealias SteadyTimerCallback = (SteadyTimerEvent) -> Void


/// Manages a steady-timer callback.
///
/// A Timer should always be created through a call to NodeHandle.createSteadyTimer(), or
/// copied from one that was. Once all copies of a specific SteadyTimer go out of scope, the
/// callback associated with that handle will stop being called.


public final class SteadyTimer {
    private static var manager = TimerManager<SteadyTime,WallDuration,SteadyTimerEvent>()

    private var started: Bool = false
    private var timerHandle: TimerHandle = .none

    private var period: WallDuration
    let callback: SteadyTimerCallback
    let callbackQueue: CallbackQueueInterface
    let trackedObject: AnyObject?
    let hasTrackedObject: Bool
    let oneShot: Bool

    internal init(period: WallDuration,
                  callback: @escaping SteadyTimerCallback,
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
            timerHandle = SteadyTimer.manager.add(period: period,
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
            SteadyTimer.manager.remove(timerHandle: timerHandle)
            timerHandle = .none
        }
    }

    func hasPending() -> Bool {
        if !isValid() || timerHandle.isNone {
            return false
        }

        return SteadyTimer.manager.hasPending(handle: timerHandle)
    }

    func setPeriod(period: WallDuration, reset: Bool = true) {
        self.period = period
        SteadyTimer.manager.setPeriod(handle: timerHandle, period: period, reset: reset)
    }

}




public struct SteadyTimerEvent: Event {
    static func createEvent(lastExpected: SteadyTime, lastExpired: SteadyTime, lastReal: SteadyTime, currentExpected: SteadyTime, currentExpired: SteadyTime, currentReal: SteadyTime) -> SteadyTimerEvent {
        return SteadyTimerEvent(lastExpected: lastExpected, lastExpired: lastExpired, lastReal: lastReal, currentExpected: currentExpected, currentExpired: currentExpired, currentReal: currentReal)

    }

    let lastExpected: SteadyTime
    var lastExpired: SteadyTime
    let lastReal: SteadyTime
    let currentExpected: SteadyTime
    var currentExpired: SteadyTime
    let currentReal: SteadyTime
}

