//
//  Rate.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-15.
//

import Foundation

public final class Rate {
    var start: Time
    public let expectedCycleTime: RosDuration
    var actualCycleTime: RosDuration
    
    public init(frequency: Double) {
        start = Time.now
        expectedCycleTime = RosDuration(seconds: 1.0 / frequency)
        actualCycleTime = RosDuration()
    }
    
    public init(duration: RosDuration) {
        start = Time.now
        expectedCycleTime = duration
        actualCycleTime = RosDuration()
    }
    
    @discardableResult
    public func sleep() async -> Bool {
        var expectedEnd = start + expectedCycleTime
        let actualEnd = Time.now
        if actualEnd < start {
            expectedEnd = actualEnd + expectedCycleTime
        }
        let sleepTime = expectedEnd - actualEnd
        actualCycleTime = actualEnd - start
        start = expectedEnd
        if sleepTime <= RosDuration() {
            if actualEnd > expectedEnd + expectedCycleTime {
                start = actualEnd
            }
            return false
        }
        
        return await sleepTime.sleep()
    }
    
    public func reset() {
        start = Time.now
    }
    
    public func cycleTime() -> RosDuration {
        return actualCycleTime
    }
}

struct WallRate {
    var start: WallTime
    let expectedCycleTime: WallDuration
    var actualCycletime: WallDuration
    
    init(frequency: Double) {
        start = WallTime.now
        expectedCycleTime = WallDuration(seconds: 1.0 / frequency)
        actualCycletime = WallDuration()
    }
    
    init(duration: WallDuration) {
        start = WallTime.now
        expectedCycleTime = duration
        actualCycletime = WallDuration()
    }
    
    @discardableResult
    mutating func sleep() async -> Bool {
        var expectedEnd = start + expectedCycleTime
        let actualEnd = WallTime.now
        if actualEnd < start {
            expectedEnd = actualEnd + expectedCycleTime
        }
        let sleepTime = expectedEnd - actualEnd
        actualCycletime = actualEnd - start
        start = expectedEnd
        if sleepTime <= WallDuration() {
            if actualEnd > expectedEnd + expectedCycleTime {
                start = actualEnd
            }
            return false
        }
        
        return await sleepTime.sleep()
    }
    
    mutating func reset() {
        start = WallTime.now
    }
    
    func cycleTime() -> WallDuration {
        return actualCycletime
    }
    
}

