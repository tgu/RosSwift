//
//  Rate.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-15.
//

import Foundation

public struct Rate {
    var start: Time
    let expectedCycleTime: Duration
    var actualCycleTime: Duration

    public init(frequency: Double) {
        start = Time.now
        expectedCycleTime = Duration(seconds: 1.0 / frequency)
        actualCycleTime = Duration()
    }

    public init(duration: Duration) {
        start = Time.now
        expectedCycleTime = duration
        actualCycleTime = Duration()
    }

    @discardableResult
    public mutating func sleep() -> Bool {
        var expectedEnd = start + expectedCycleTime
        let actualEnd = Time.now
        if actualEnd < start {
            expectedEnd = actualEnd + expectedCycleTime
        }
        let sleepTime = expectedEnd - actualEnd
        actualCycleTime = actualEnd - start
        start = expectedEnd
        if sleepTime <= Duration() {
            if actualEnd > expectedEnd + expectedCycleTime {
                start = actualEnd
            }
            return false
        }

        return sleepTime.sleep()
    }

    public mutating func reset() {
        start = Time.now
    }

    public func cycleTime() -> Duration {
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
    mutating func sleep() -> Bool {
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

        return sleepTime.sleep()
    }

    mutating func reset() {
        start = WallTime.now
    }

    func cycleTime() -> WallDuration {
        return actualCycletime
    }

}

