//
//  Time.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-10.
//

import BinaryCoder
import Foundation
import Atomics


/// Time representation. May either represent wall clock time or ROS clock time.
///
/// `TimeBase` provides most of its functionality.

public struct Time: TimeBase {

    public let nanoseconds: UInt64

    internal static let useSimTime = ManagedAtomic(true)
    public static let gStopped = ManagedAtomic(false)
    public static let gInitialized = ManagedAtomic(false)
    public static var simTime = Time()
    public static let simTimeQueue = DispatchQueue(label: "g_sim_time_mutex")
    public static let max = Time(nanosec: UInt64.max)
    public static let min = Time(nanosec: 1)

    public init(nanosec: UInt64) {
        nanoseconds = nanosec
    }

    public static func initialize() {
        gStopped.store(false, ordering: .relaxed)
        useSimTime.store(false, ordering: .relaxed)
        gInitialized.store(true, ordering: .relaxed)
    }

    public static func shutDown() {
        gStopped.store(true, ordering: .relaxed)
    }


    public static var isSimTime: Bool {
        return useSimTime.load(ordering: .relaxed)
    }

    public static var isSystemTime: Bool {
        return !isSimTime
    }

    /// Returns whether or not the current time is valid.
    /// Time is valid if it is non-zero.

    public static var isValid: Bool {
        return !useSimTime.load(ordering: .relaxed) || !Time.simTimeQueue.sync { Time.simTime.isZero }
    }

    /// Retrieve the current time. If ROS clock time is in use,
    /// this returns the time according to the ROS clock.
    /// Otherwise returns the current wall clock time.

    public static var now: Time {
        guard Time.gInitialized.load(ordering: .relaxed) else {
            fatalError("Cannot use Time.now() before the first NodeHandle has been created or Ros.start()" +
                " has been called. If this is a standalone app or test that just uses Time and does not" +
                " communicate over ROS, you may also call Time.initialize()")
        }

        if Time.useSimTime.load(ordering: .relaxed) {
            return Time.simTimeQueue.sync {
                Time.simTime
            }
        }

        let time = walltime()
        return Time(sec: time.sec, nsec: time.nsec)
    }

    public static func setNow(_ now: Time) {
        simTimeQueue.sync {
            simTime = now
            useSimTime.store(true, ordering: .relaxed)
        }
    }

    /// Wait for time to become valid, with timeout

    public static func waitForValid(timeout: WallDuration = WallDuration()) -> Bool {
        let start = WallTime.now
        while !isValid && !gStopped.load(ordering: .relaxed) {
            _ = WallDuration(seconds: 0.01).sleep()
            if timeout > WallDuration(sec: 0, nsec: 0) && WallTime.now - start > timeout {
                return false
            }
        }
        if gStopped.load(ordering: .relaxed) {
            return false
        }
        return true
    }

    public static func + (lhs: Time, rhs: RosDuration) -> Time {
        return Time(nanosec: lhs.nanoseconds + UInt64(rhs.nanoseconds))
    }

    public static func - (lhs: Time, rhs: Time) -> RosDuration {
        return RosDuration(nanosec: Int64(lhs.nanoseconds) - Int64(rhs.nanoseconds))
    }

}

