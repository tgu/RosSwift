//
//  Timer.swift
//  RosTime
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

import Foundation
import RosTime
import NIOConcurrencyHelpers

typealias TimerCallback = (TimerEvent) -> Void

enum TimerHandle: Equatable, Hashable {
    case none
    case some(UInt32)

    var isNone: Bool {
        switch self {
        case .none:
            return true
        default:
            return false
        }
    }

    static let id = Atomic<UInt32>(value: 0)

    static func getNextHandler() -> TimerHandle {
        return TimerHandle.some(TimerHandle.id.add(1))
    }

}

/// Manages a timer callback.
///
/// A Timer should always be created through a call to NodeHandle::createTimer(), or
/// copied from one that was. Once all copies of a specific Timer go out of scope, the
/// callback associated with that handle will stop being called.



public final class Timer {
    private static var manager = TimerManager<Time,Duration,TimerEvent>()
    
    private var started: Bool = false
    private var timerHandle: TimerHandle = .none

    private var period: Duration
    let callback: TimerCallback
    let callbackQueue: CallbackQueueInterface
    let trackedObject: AnyObject?
    let hasTrackedObject: Bool
    let oneShot: Bool

    internal init(period: Duration,
                  callback: @escaping TimerCallback,
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
            timerHandle = Timer.manager.add(period: period,
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
            Timer.manager.remove(timerHandle: timerHandle)
            timerHandle = .none
        }
    }

    func hasPending() -> Bool {
        if !isValid() || timerHandle.isNone {
            return false
        }

        return Timer.manager.hasPending(handle: timerHandle)
    }

    func setPeriod(period: Duration, reset: Bool = true) {
        self.period = period
        Timer.manager.setPeriod(handle: timerHandle, period: period, reset: reset)
    }

}


