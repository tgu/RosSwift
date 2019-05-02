//
//  TimerEvent.swift
//  RosTime
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

protocol Event {
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


import Foundation
import RosTime

/// Structure passed as a parameter to the callback invoked by a `Timer`

public struct TimerEvent: Event {
    typealias EventTime = Time
    static func createEvent(lastExpected: Time, lastExpired: Time, lastReal: Time, currentExpected: Time, currentExpired: Time, currentReal: Time) -> TimerEvent {
        return TimerEvent(lastExpected: lastExpected, lastExpired: lastExpired, lastReal: lastReal, currentExpected: currentExpected, currentExpired: currentExpired, currentReal: currentReal)
    }



    /// In a perfect world, this is when the last callback should have happened.
    let lastExpected: Time

    /// When the last timer actually expired and the callback was added to the queue.
    let lastExpired: Time

    /// When the last callback actually happened.
    let lastReal: Time

    /// In a perfect world, this is when the current callback should be happening
    let currentExpected: Time

    /// When the current timer actually expired and the callback was added to the queue
    let currentExpired: Time

    /// This is when the current callback was actually called (Time::now() as of the beginning of the callback)
    let currentReal: Time

    struct Profile {
        /// How long the last callback ran for.
        let lastDuration: WallDuration
    }
}
