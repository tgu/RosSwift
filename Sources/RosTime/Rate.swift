//
//  Rate.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-15.
//

import Foundation

extension RosTime {


struct Rate {
    var start_ : Time
    let expected_cycle_time_ : Duration
    var actual_cycle_time_ : Duration

    init(frequency: Double) {
        start_ = Time.now()
        expected_cycle_time_ = Duration(seconds: 1.0 / frequency)
        actual_cycle_time_ = Duration()
    }

    init(duration: Duration) {
        start_ = Time.now()
        expected_cycle_time_ = duration
        actual_cycle_time_ = Duration()
    }

    mutating func sleep() -> Bool {
        var expected_end = start_ + expected_cycle_time_
        let actual_end = Time.now()
        if actual_end < start_ {
            expected_end = actual_end + expected_cycle_time_
        }
        let sleep_time = expected_end - actual_end
        actual_cycle_time_ = actual_end - start_
        start_ = expected_end
        if sleep_time <= Duration() {
            if actual_end > expected_end + expected_cycle_time_ {
                start_ = actual_end
            }
            return false
        }

        return sleep_time.sleep()
    }

    mutating func reset() {
        start_ = Time.now()
    }

    func cycleTime() -> Duration {
        return actual_cycle_time_
    }

    func expectedCycleTime() -> Duration {
        return expected_cycle_time_
    }

}


struct WallRate {
    var start_ : WallTime
    let expected_cycle_time_ : WallDuration
    var actual_cycle_time_ : WallDuration

    init(frequency: Double) {
        start_ = WallTime.now()
        expected_cycle_time_ = WallDuration(seconds: 1.0 / frequency)
        actual_cycle_time_ = WallDuration()
    }

    init(duration: WallDuration) {
        start_ = WallTime.now()
        expected_cycle_time_ = duration
        actual_cycle_time_ = WallDuration()
    }

    mutating func sleep() -> Bool {
        var expected_end = start_ + expected_cycle_time_
        let actual_end = WallTime.now()
        if actual_end < start_ {
            expected_end = actual_end + expected_cycle_time_
        }
        let sleep_time = expected_end - actual_end
        actual_cycle_time_ = actual_end - start_
        start_ = expected_end
        if sleep_time <= WallDuration() {
            if actual_end > expected_end + expected_cycle_time_ {
                start_ = actual_end
            }
            return false
        }

        return sleep_time.sleep()
    }

    mutating func reset() {
        start_ = WallTime.now()
    }

    func cycleTime() -> WallDuration {
        return actual_cycle_time_
    }

    func expectedCycleTime() -> WallDuration {
        return expected_cycle_time_
    }

}

}
