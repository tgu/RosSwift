//
//  DurationBase.swift
//  RosTime
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

import BinaryCoder
import Foundation


public protocol BasicDurationBase: Sendable {
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
    
    func sleep() async -> Bool
}

public extension BasicDurationBase {
    @inlinable
    static func milliseconds(_ ms: Int32) -> Self {
        .init(milliseconds: ms)
    }
    
    @inlinable
    func seconds(_ s: TimeInterval) -> Self {
        .init(seconds: s)
    }
    
}

public protocol DurationBase: BasicDurationBase, Comparable, BinaryCodable {}

public extension DurationBase {
    
    @inlinable
    static func < (lhs: Self, rhs: Self) -> Bool {
        return lhs.toNSec() < rhs.toNSec()
    }
    
    @inlinable
    static func == (lhs: Self, rhs: Self) -> Bool {
        return lhs.toNSec() == rhs.toNSec()
    }
    
    @inlinable
    var sec: Int32 {
        return Int32(nanoseconds / 1_000_000_000 )
    }
    
    @inlinable
    var nsec: Int32 {
        return Int32(nanoseconds % 1_000_000_000 )
    }
    
    @inlinable
    var time: timespec {
        return timespec(tv_sec: Int(sec), tv_nsec: Int(nsec))
    }
    
    @inlinable
    init() {
        self.init(nanosec: 0)
    }
    
    @inlinable
    init(sec: Int32, nsec: Int32) {
        let nano = Int64(sec) * 1_000_000_000 + Int64(nsec)
        self.init(nanosec: nano)
    }
    
    @inlinable
    init(milliseconds: Int32) {
        let nano = Int64(milliseconds) * 1_000_000
        self.init(nanosec: nano)
    }
    
    @inlinable
    init(seconds: TimeInterval) {
        let nano = Int64( floor(seconds * 1_000_000_000 ))
        self.init(nanosec: nano)
    }
    
    @inlinable
    func isZero() -> Bool {
        return nanoseconds == 0
    }
    
    @inlinable
    func toNSec() -> Int64 {
        return nanoseconds
    }
    
    @inlinable
    func toSec() -> TimeInterval {
        return TimeInterval(nanoseconds) * 1e-9
    }
}
