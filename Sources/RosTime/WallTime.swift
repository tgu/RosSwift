//
//  WallTime.swift
//  RosTime
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

import Foundation


public struct WallTime: TimeBase {
    public let nanoseconds: UInt64

    public init(nanosec: UInt64) {
        nanoseconds = nanosec
    }

    public static var now: WallTime {
        let time = walltime()
        return WallTime(sec: time.sec, nsec: time.nsec)
    }
}


internal func walltime() -> (sec: UInt32, nsec: UInt32) {
    var start = timespec()
    clock_gettime(CLOCK_REALTIME, &start)
    return (UInt32(start.tv_sec), UInt32(start.tv_nsec))
}
