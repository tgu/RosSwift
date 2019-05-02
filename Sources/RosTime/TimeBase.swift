//
//  TimeBase.swift
//  RosTime
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

import BinaryCoder
import Foundation

/// Protocol for Time implementations.

public protocol TimeBase: Comparable, BinaryCodable {

    var nanoseconds: UInt64 {get}

    init(nanosec: UInt64)

    static func isSystemTime() -> Bool
    static var now: Self { get }
}

// deafult implementations

public extension TimeBase {

    var sec: UInt32 {
        return UInt32(nanoseconds / 1_000_000_000)
    }

    var nsec: UInt32 {
        return UInt32(nanoseconds % 1_000_000_000)
    }

    init() {
        self.init(nanosec: 0)
    }

    init(sec: UInt32, nsec: UInt32) {
        let nano = UInt64(sec) * 1_000_000_000 + UInt64(nsec)
        self.init(nanosec: nano)
    }

    init(seconds: TimeInterval) {
        let nano = UInt64( floor(seconds * 1_000_000_000) )
        self.init(nanosec: nano)
    }

    func isZero() -> Bool {
        return nanoseconds == 0
    }

    func toSec() -> TimeInterval {
        return TimeInterval(nanoseconds) * 1e-9
    }

    static func isSystemTime() -> Bool {
        return true
    }

    static func += (lhs: inout Self, rhs: BasicDurationBase) {
        lhs = lhs + rhs
    }

    static func < (lhs: Self, rhs: Self) -> Bool {
        return lhs.nanoseconds < rhs.nanoseconds
    }

    static func == (lhs: Self, rhs: Self) -> Bool {
        return lhs.nanoseconds == rhs.nanoseconds
    }

    static func distantFuture() -> Self {
        return Self(nanosec: UInt64.max)
    }

    static func + (lhs: Self, rhs: BasicDurationBase) -> Self {
        return Self(nanosec: UInt64(Int64(lhs.nanoseconds) + rhs.nanoseconds))
    }

    static func - (lhs: Self, rhs: Self) -> WallDuration {
        return WallDuration(nanosec: Int64(lhs.nanoseconds) - Int64(rhs.nanoseconds))
    }
}
