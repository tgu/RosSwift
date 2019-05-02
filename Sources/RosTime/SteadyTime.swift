//
//  SteadyTime.swift
//  RosTime
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

import Foundation

public struct SteadyTime: TimeBase {
    public let nanoseconds: UInt64

    public init(nanosec: UInt64) {
        nanoseconds = nanosec
    }

    public static var now: SteadyTime {
        var start = timespec()
        clock_gettime(CLOCK_MONOTONIC, &start)
        return SteadyTime(sec: UInt32(start.tv_sec), nsec: UInt32(start.tv_nsec))
    }

    public static func - (lhs: SteadyTime, rhs: SteadyTime) -> WallDuration {
        return WallDuration(nanosec: Int64(lhs.nanoseconds) - Int64(rhs.nanoseconds))
    }

}
