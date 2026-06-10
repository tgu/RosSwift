//
//  Duration.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-15.
//

import BinaryCoder
import Foundation


fileprivate func wallSleep(sec: UInt32, nsec: UInt32) async -> Bool {
    let nanoseconds = UInt64(sec) * 1_000_000_000 + UInt64(nsec)
    try? await Task.sleep(nanoseconds: nanoseconds)
    return !Time.gStopped.load(ordering: .relaxed)
}


public struct RosDuration: DurationBase, Sendable {
    public let nanoseconds: Int64
    
    public init(nanosec: Int64) {
        nanoseconds = nanosec
    }
    
    @discardableResult
    public func sleep() async -> Bool {
        if !Time.useSimTime.load(ordering: .relaxed) {
            return await wallSleep(sec: UInt32(sec), nsec: UInt32(nsec))
        }
        
        var start = Time.now
        var end = start + self
        if start.isZero {
            end = Time(nanosec: UInt64(Int64.max))
        }
        
        var didSleep = false
        while !Time.gStopped.load(ordering: .relaxed) && Time.now < end {
            _ = await wallSleep(sec: 0, nsec: 1_000_000)
            didSleep = true
            if start.isZero {
                start = Time.now
                end = start + self
            }
            if Time.now < start {
                return false
            }
        }
        return didSleep && !Time.gStopped.load(ordering: .relaxed)
    }
    
}

public struct WallDuration: DurationBase, Sendable {
    public var nanoseconds: Int64
    
    public init(nanosec: Int64) {
        nanoseconds = nanosec
    }
    
    @discardableResult
    public func sleep() async -> Bool {
        _ = try? await Task.sleep(nanoseconds: UInt64(nanoseconds))
        return true
    }
    
    public static func + (lhs: WallDuration, rhs: WallDuration) -> WallDuration {
        return WallDuration(nanosec: lhs.nanoseconds + rhs.nanoseconds)
    }
}
