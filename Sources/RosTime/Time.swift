//
//  Time.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-10.
//

import BinaryCoder
import Foundation

public protocol TimeBase: Comparable, BinaryCodable {
}

public struct RosTime {
    static func rosWalltime() -> (sec: UInt32, nsec: UInt32) {
        var start = timespec()
        clock_gettime(CLOCK_REALTIME, &start)
        return (UInt32(start.tv_sec), UInt32(start.tv_nsec))
    }

    static func rosWallsleep(sec: UInt32, nsec: UInt32) -> Bool {
        var req = timespec(tv_sec: Int(sec), tv_nsec: Int(nsec))
        var rem = timespec(tv_sec: 0, tv_nsec: 0)
        while nanosleep(&req, &rem) != 0 && !Time.gStopped {
            req = rem

        }
        return !Time.gStopped
    }

    static func normalizeSecNSec(_ sec: inout UInt64, _ nsec: inout UInt64) {
        let nsecPart = nsec % 1_000_000_000
        let secPart = nsec / 1_000_000_000

        if sec + secPart > UInt32.max {
            sec = UInt64(UInt32.max)
            nsec = 0
        } else {
            sec += secPart
            nsec = nsecPart
        }
    }

    static func normalizeSecNSec(_ sec: inout UInt32, _ nsec: inout UInt32) {
        var sec64 = UInt64(sec)
        var nsec64 = UInt64(nsec)

        normalizeSecNSec(&sec64, &nsec64)

        sec = UInt32(sec64)
        nsec = UInt32(nsec64)
    }

    static func normalizeSecNSecSigned(_ sec: inout Int64, _ nsec: inout Int64) {
        var nsecPart = nsec % 1000000000
        var secPart = sec + nsec / 1000000000
        if nsecPart < 0 {
            nsecPart += 1000000000
            secPart -= 1
        }

        if secPart < Int32.min || secPart > Int32.max {
            fatalError("normalizeSecNSecSigned of \(sec):\(nsec) failed")
        }

        sec = secPart
        nsec = nsecPart
    }

    static func normalizeSecNSecSigned(_ sec: inout Int32, _ nsec: inout Int32) {
        var sec64 = Int64(sec)
        var nsec64 = Int64(nsec)

        normalizeSecNSecSigned(&sec64, &nsec64)

        sec = Int32(sec64)
        nsec = Int32(nsec64)
    }

    public struct TimerEvent {
        let lastExpected: Time
        let lastReal: Time
        let currentExpected: Time
        let currentReal: Time

        struct Profile {
            let lastDuration: WallDuration
        }
    }

    public struct SteadyTimerEvent {
        let lastExpected: SteadyTime
        let lastReal: SteadyTime
        let currentExpected: SteadyTime
        let currentReal: SteadyTime

        struct Profile {
            let lastDuration: WallDuration
        }
    }

    public struct WallTimerEvent {
        let lastExpected: WallTime
        let lastReal: WallTime
        let currentExpected: WallTime
        let currentReal: WallTime

        struct Profile {
            let lastDuration: WallDuration
        }
    }

    public final class WallTime: TimeBase {

        public static func now() -> WallTime {
            let time = rosWalltime()
            return WallTime(sec: time.sec, nsec: time.nsec)
        }

        public static func + (lhs: WallTime, rhs: WallDuration) -> WallTime {
            return WallTime(nanosec: lhs.toNSec() + UInt64(rhs.toNSec()))
        }

        public static func - (lhs: WallTime, rhs: WallTime) -> WallDuration {
            return WallDuration(nanosec: Int64(lhs.toNSec()) - Int64(rhs.toNSec()))
        }

    }

    public class TimeBase: Comparable, BinaryCodable {

        public final var nanoseconds: UInt64

        public final var sec: UInt32 {
            return UInt32(nanoseconds / 1_000_000_000)
        }

        public final var nsec: UInt32 {
            return UInt32(nanoseconds % 1_000_000_000)
        }

        public init() {
            nanoseconds = 0
        }

        public init(sec: UInt32, nsec: UInt32) {
            self.nanoseconds = UInt64(sec) * 1_000_000_000 + UInt64(nsec)
        }

        public init(seconds: TimeInterval) {
            nanoseconds = UInt64( floor(seconds * 1_000_000_000) )
        }

        public init(nanosec: UInt64) {
            nanoseconds = nanosec
        }

        public final func isZero() -> Bool {
            return nanoseconds == 0
        }

        public final func toNSec() -> UInt64 {
            return nanoseconds
        }

        public final func toSec() -> TimeInterval {
            return TimeInterval(sec) * 1e-9
        }

        public static let distantFuture = TimeBase(nanosec: UInt64.max)

        public static func < (lhs: TimeBase, rhs: TimeBase) -> Bool {
            return lhs.nanoseconds < rhs.nanoseconds
        }

        public static func == (lhs: TimeBase, rhs: TimeBase) -> Bool {
            return lhs.nanoseconds == rhs.nanoseconds
        }
    }

    public class Time: TimeBase {

        public static var useSimTime = true
        public static var gStopped = false
        public static var gInitialized = false
        public static var simTime = Time()
        public static var simTimeQueue = DispatchQueue(label: "g_sim_time_mutex")

        public static func initialize() {
            gStopped = false
            useSimTime = false
            gInitialized = true
        }

        public static func shutDown() {
            gStopped = true
        }

        public static func isValid() -> Bool {
            return !useSimTime || simTime.isZero()
        }

        public static func now() -> Time {
            guard Time.gInitialized else {
                fatalError("Cannot use ros::Time::now() before the first NodeHandle has been created or ros::start()" +
                    " has been called. If this is a standalone app or test that just uses ros::Time and does not" +
                    " communicate over ROS, you may also call ros::Time::init()")
            }

            if Time.useSimTime {
                return Time.simTimeQueue.sync {
                    Time.simTime
                }
            }

            let time = rosWalltime()
            return Time(sec: time.sec, nsec: time.nsec)
        }

        public static func setNow(_ now: Time) {
            simTimeQueue.sync {
                simTime = now
                useSimTime = true
            }
        }

        public static func waitForValid(timeout: WallDuration) -> Bool {
            let start = WallTime.now()
            while !isValid() && !gStopped {
                _ = WallDuration(seconds: 0.01).sleep()
                if timeout > WallDuration(sec: 0, nsec: 0) && WallTime.now() - start > timeout {
                    return false
                }
            }
            if gStopped {
                return false
            }
            return true
        }

        public static func + (lhs: Time, rhs: Duration) -> Time {
            return Time(nanosec: lhs.toNSec() + UInt64(rhs.toNSec()))
        }
        public static func - (lhs: Time, rhs: Time) -> Duration {
            return Duration(nanosec: Int64(lhs.toNSec()) - Int64(rhs.toNSec()))
        }

    }

    public class SteadyTime: TimeBase {

        public static func now() -> SteadyTime {
            var start = timespec()
            clock_gettime(CLOCK_MONOTONIC, &start)
            return SteadyTime(sec: UInt32(start.tv_sec), nsec: UInt32(start.tv_nsec))
        }

    }
}
