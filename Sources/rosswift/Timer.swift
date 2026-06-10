//
//  Timer.swift
//  RosTime
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

import Foundation
import RosTime
import Atomics

public protocol TrackableObject: AnyObject, Sendable {}
public typealias TimerCallback = @Sendable (TimerEvent) -> Void

typealias TimerHandle = UUID

/*
 enum TimerHandle: Equatable, Hashable, Sendable {
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
 
 static let id = ManagedAtomic<UInt32>(0)
 
 static func getNextHandler() -> TimerHandle {
 return TimerHandle.some(TimerHandle.id.loadThenWrappingIncrement(ordering: .relaxed))
 }
 
 }
 */

/// Manages a timer callback.
///
/// A Timer should always be created through a call to NodeHandle::createTimer(), or
/// copied from one that was. Once all copies of a specific Timer go out of scope, the
/// callback associated with that handle will stop being called.


public actor Timer {
    private static let manager = TimerManager<Time,TimerEvent>()
    
    private var started = false
    private var timerHandle: TimerHandle? = nil
    
    private var period: RosDuration
    let callback: TimerCallback
    let callbackQueue: AsyncCallbackQueue
    weak var trackedObject: TrackableObject?
    let hasTrackedObject: Bool
    let oneShot: Bool
    
    internal init(period: RosDuration,
                  callback: @escaping TimerCallback,
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
        if started, let handle = timerHandle {
            Task {
                await Timer.manager.remove(timerHandle: handle)
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
            timerHandle = await Timer.manager.add(period: period,
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
                await Timer.manager.remove(timerHandle: handle)
                timerHandle = .none
            }
        }
    }
    
    func hasPending() async -> Bool {
        if !isValid() {
            return false
        } else if let handle = timerHandle {
            return await Timer.manager.hasPending(handle: handle)
        } else {
            return false
        }
    }
    
    func setPeriod(period: RosDuration, reset: Bool = true) async  {
        self.period = period
        if let handle = timerHandle {
            await Timer.manager.setPeriod(handle: handle, period: period, reset: reset)
        }
    }
    
}


