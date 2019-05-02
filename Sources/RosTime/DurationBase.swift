//
//  DurationBase.swift
//  RosTime
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

import BinaryCoder
import Foundation


public protocol BasicDurationBase {
    var nanoseconds: Int64 { get }
    var sec: Int32 { get }
    var nsec: Int32 { get }
    var time: timespec { get }

    init()
    init(sec: Int32, nsec: Int32)
    init(milliseconds: Int32)
    init(seconds: TimeInterval)
    init(nanosec: Int64)

    func isZero() -> Bool
    func toNSec() -> Int64
    func toSec() -> TimeInterval

    func sleep() -> Bool
}



public protocol DurationBase: BasicDurationBase, Comparable, BinaryCodable {

}

public extension DurationBase {

    static func < (lhs: Self, rhs: Self) -> Bool {
        return lhs.toNSec() < rhs.toNSec()
    }

    static func == (lhs: Self, rhs: Self) -> Bool {
        return lhs.toNSec() == rhs.toNSec()
    }

    var sec: Int32 {
        return Int32(nanoseconds / 1_000_000_000 )
    }

    var nsec: Int32 {
        return Int32(nanoseconds % 1_000_000_000 )
    }

    var time: timespec {
        return timespec(tv_sec: Int(sec), tv_nsec: Int(nsec))
    }

    init() {
        self.init(nanosec: 0)
    }

    init(sec: Int32, nsec: Int32) {
        let nano = Int64(sec) * 1_000_000_000 + Int64(nsec)
        self.init(nanosec: nano)
    }

    init(milliseconds: Int32) {
        let nano = Int64(milliseconds) * 1_000_000
        self.init(nanosec: nano)
    }

    init(seconds: TimeInterval) {
        let nano = Int64( floor(seconds * 1_000_000_000 ))
        self.init(nanosec: nano)
    }

    func isZero() -> Bool {
        return nanoseconds == 0
    }

    func toNSec() -> Int64 {
        return nanoseconds
    }

    func toSec() -> TimeInterval {
        return TimeInterval(nanoseconds) * 1e-9
    }
}
