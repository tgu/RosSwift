//
//  Time.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-10.
//

import BinaryCoder
import Foundation

/// Time representation. May either represent wall clock time or ROS clock time.
///
/// `TimeBase` provides most of its functionality.

public struct Time: TimeBase {

    public let nanoseconds: UInt64

    internal static var useSimTime = true
    public static var gStopped = false
    public static var gInitialized = false
    public static var simTime = Time()
    public static var simTimeQueue = DispatchQueue(label: "g_sim_time_mutex")

    public init(nanosec: UInt64) {
        nanoseconds = nanosec
    }

    public static func initialize() {
        gStopped = false
        useSimTime = false
        gInitialized = true
    }

    public static func shutDown() {
        gStopped = true
    }


    public static func isSimTime() -> Bool {
        return useSimTime
    }

    public static func isSystemTime() -> Bool {
        return !isSimTime()
    }

    /// Returns whether or not the current time is valid.
    /// Time is valid if it is non-zero.

    public static func isValid() -> Bool {
        return !useSimTime || simTime.isZero()
    }

    /// Retrieve the current time. If ROS clock time is in use,
    /// this returns the time according to the ROS clock.
    /// Otherwise returns the current wall clock time.

    public static var now: Time {
        guard Time.gInitialized else {
            fatalError("Cannot use Time.now() before the first NodeHandle has been created or Ros.start()" +
                " has been called. If this is a standalone app or test that just uses Time and does not" +
                " communicate over ROS, you may also call Time.initialize()")
        }

        if Time.useSimTime {
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
            useSimTime = true
        }
    }

    /// Wait for time to become valid, with timeout

    public static func waitForValid(timeout: WallDuration = WallDuration()) -> Bool {
        let start = WallTime.now
        while !isValid() && !gStopped {
            _ = WallDuration(seconds: 0.01).sleep()
            if timeout > WallDuration(sec: 0, nsec: 0) && WallTime.now - start > timeout {
                return false
            }
        }
        if gStopped {
            return false
        }
        return true
    }

    public static func + (lhs: Time, rhs: Duration) -> Time {
        return Time(nanosec: lhs.nanoseconds + UInt64(rhs.nanoseconds))
    }

    public static func - (lhs: Time, rhs: Time) -> Duration {
        return Duration(nanosec: Int64(lhs.nanoseconds) - Int64(rhs.nanoseconds))
    }

}

