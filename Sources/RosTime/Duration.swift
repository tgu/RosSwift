//
//  Duration.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-15.
//

import BinaryCoder
import Foundation

extension RosTime {
    public class DurationBase: Comparable, BinaryCodable {
        public static func < (lhs: DurationBase, rhs: DurationBase) -> Bool {
            return lhs.toNSec() < rhs.toNSec()
        }

        public static func == (lhs: DurationBase, rhs: DurationBase) -> Bool {
            return lhs.toNSec() == rhs.toNSec()
        }

        public final var nanoseconds: Int64

        public final var sec: Int32 {
            return Int32(nanoseconds / 1_000_000_000 )
        }

        public final var nsec: Int32 {
            return Int32(nanoseconds % 1_000_000_000 )
        }

        public final var time: timespec {
            return timespec(tv_sec: Int(sec), tv_nsec: Int(nsec))
        }

        public init() {
            nanoseconds = 0
        }

        public init(sec: Int32, nsec: Int32) {
            nanoseconds = Int64(sec) * 1_000_000_000 + Int64(nsec)
        }

        public init(milliseconds: Int32) {
            nanoseconds = Int64(milliseconds) * 1_000_000
        }

        public init(seconds: TimeInterval) {
            nanoseconds = Int64( floor(seconds * 1_000_000_000 ))
        }

        public init(nanosec: Int64) {
            nanoseconds = nanosec
        }

        public final func isZero() -> Bool {
            return nanoseconds == 0
        }

        public final func toNSec() -> Int64 {
            return nanoseconds
        }

        public final func toSec() -> TimeInterval {
            return TimeInterval(nanoseconds) * 1e-9
        }

        public func sleep() -> Bool {
            return true
        }

    }

    public final class Duration: DurationBase {

        @discardableResult
        public override func sleep() -> Bool {
            if !Time.useSimTime {
                return rosWallsleep(sec: UInt32(sec), nsec: UInt32(nsec))
            }

            var start = Time.now()
            var end = start + self
            if start.isZero() {
                end = Time(nanosec: UInt64.max)
            }

            var didSleep = false
            while !Time.gStopped && Time.now() < end {
                _ = rosWallsleep(sec: 0, nsec: 1_000_000)
                didSleep = true
                if start.isZero() {
                    start = Time.now()
                    end = start + self
                }
                if Time.now() < start {
                    return false
                }
            }
            return didSleep && !Time.gStopped
        }

    }

    public final class WallDuration: DurationBase {

        @discardableResult
        public override func sleep() -> Bool {
            return rosWallsleep(sec: UInt32(sec), nsec: UInt32(nsec))
        }

        public static func + (lhs: WallDuration, rhs: WallDuration) -> WallDuration {
            return WallDuration(nanosec: lhs.nanoseconds + rhs.nanoseconds)
        }
    }

}
