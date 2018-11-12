//
//  Duration.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-15.
//

import Foundation
import BinaryCoder

extension RosTime {
    public class DurationBase: Comparable, BinaryCodable {
    public static func < (lhs: DurationBase, rhs: DurationBase) -> Bool {
        return lhs.toNSec() < rhs.toNSec()
    }

    public static func == (lhs: DurationBase, rhs: DurationBase) -> Bool {
        return lhs.toNSec() == rhs.toNSec()
    }

    public var nanoseconds : Int64

    public var sec: Int32 {
        return Int32(nanoseconds / 1_000_000_000 )
    }

    public var nsec: Int32 {
        return Int32(nanoseconds % 1_000_000_000 )
    }

    public var time: timespec {
        return timespec(tv_sec: Int(sec), tv_nsec: Int(nsec))
    }

    public init() {
        nanoseconds = 0
    }

    public init(sec: Int32, nsec: Int32) {
        nanoseconds =  Int64(sec) * 1_000_000_000 + Int64(nsec)
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

    public func isZero() -> Bool {
        return nanoseconds == 0
    }

    public func toNSec() -> Int64 {
        return nanoseconds
    }

    public func toSec() -> TimeInterval {
        return TimeInterval(nanoseconds)*1e-9
    }



}

public class Duration: DurationBase {

    @discardableResult
    public func sleep() -> Bool {
        if !Time.g_use_sim_time {
            return ros_wallsleep(sec: UInt32(sec),nsec: UInt32(nsec))
        }

        var start = Time.now()
        var end = start + self
        if start.isZero() {
            end = Time(nanosec: UInt64.max)
        }

        var rc = false
        while !Time.gStopped && Time.now() < end {
            let _ = ros_wallsleep(sec: 0, nsec: 1_000_000)
            rc = true
            if start.isZero() {
                start = Time.now()
                end = start + self
            }
            if Time.now() < start {
                return false
            }
        }
        return rc && !Time.gStopped
    }

}


public class WallDuration: DurationBase {

    @discardableResult
    public func sleep() -> Bool {
        return ros_wallsleep(sec: UInt32(sec),nsec: UInt32(nsec))
    }

    public static func + (lhs: WallDuration, rhs: WallDuration) -> WallDuration {
        return WallDuration(nanosec: lhs.nanoseconds + rhs.nanoseconds)
    }
}


}
