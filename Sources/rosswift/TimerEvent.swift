//
//  TimerEvent.swift
//  RosTime
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

public protocol Event {
    associatedtype EventTime: TimeBase
    var lastExpected: EventTime { get }
    var lastExpired: EventTime { get }
    var lastReal: EventTime { get }
    var currentExpected: EventTime { get }
    var currentExpired: EventTime { get }
    var currentReal: EventTime { get }

    static func createEvent(lastExpected: EventTime,
                            lastExpired: EventTime,
                            lastReal: EventTime,
                            currentExpected: EventTime,
                            currentExpired: EventTime,
                            currentReal: EventTime) -> Self
}

import RosTime

/// Structure passed as a parameter to the callback invoked by a `Timer`

public struct TimerEvent: Event {
    public typealias EventTime = Time
    public static func createEvent(lastExpected: Time, lastExpired: Time, lastReal: Time, currentExpected: Time, currentExpired: Time, currentReal: Time) -> TimerEvent {
        return TimerEvent(lastExpected: lastExpected, lastExpired: lastExpired, lastReal: lastReal, currentExpected: currentExpected, currentExpired: currentExpired, currentReal: currentReal)
    }



    /// In a perfect world, this is when the last callback should have happened.
    public let lastExpected: Time

    /// When the last timer actually expired and the callback was added to the queue.
    public let lastExpired: Time

    /// When the last callback actually happened.
    public let lastReal: Time

    /// In a perfect world, this is when the current callback should be happening
    public let currentExpected: Time

    /// When the current timer actually expired and the callback was added to the queue
    public let currentExpired: Time

    /// This is when the current callback was actually called (Time::now() as of the beginning of the callback)
    public let currentReal: Time

    struct Profile {
        /// How long the last callback ran for.
        let lastDuration: WallDuration
    }
}
