//
//  TimerTest.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

import XCTest
@testable import RosSwift
@testable import RosTime
@testable import msgs
import Atomics
import rosmaster
import RosNetwork

/// Starts and run roscore during testing
/// 
class RosTest: XCTestCase {
    private static var master: rosmaster.Master? = nil
    var remap: [String:String] {
        ["__master": RosTest.master!.address]
    }
    var host: String {
        RosTest.master!.host
    }

    override static func setUp() {
        let network = RosNetwork(remappings: [:])
        master = rosmaster.Master(host: network.gHost, port: 11311, advertise: false)
        _ = try! master?.start().wait()
    }

    override static func tearDown() {
        _ = master?.stop()
    }
}



class SteadyTimerHelper {
    var expectedPeriod: WallDuration
    let oneshot: Bool
    var failed = false
    var totalCalls = 0
    var testingPeriod = false
    var callsBeforeTestingPeriod = 0
    var timer: SteadyTimer? = nil
    var lastCall = SteadyTime(nanosec: 0)

    init(node: NodeHandle, _ period: Double, oneshot: Bool = false) {
        self.oneshot = oneshot
        let d = WallDuration(seconds: period)
        self.expectedPeriod = d
        let tim = node.createSteadyTimer(period: d, trackedObject: self, callback: callback)
        self.timer = tim
    }

    func callback(event: SteadyTimerEvent) {
        let first = lastCall.isZero
        lastCall = event.currentExpired

        if !first {
            let time_error = event.currentExpired.toSec() - event.currentExpected.toSec()
            if time_error > 5.0 || time_error < -0.01 {
                ROS_ERROR("Call came at wrong time")
                failed = true
            }
        }

        if testingPeriod {
            if totalCalls == callsBeforeTestingPeriod {
                let p = WallDuration(milliseconds: 500)
                pretendWork(0.15)
                setPeriod(p)
            } else if totalCalls == callsBeforeTestingPeriod + 1 {
                let p = WallDuration(milliseconds: 250)
                pretendWork(0.15)
                setPeriod(p)
            } else if totalCalls == callsBeforeTestingPeriod + 2 {
                let p = WallDuration(milliseconds: 500)
                pretendWork(0.15)
                setPeriod(p, reset: true)
            } else if totalCalls == callsBeforeTestingPeriod + 3 {
                let p = WallDuration(milliseconds: 250)
                pretendWork(0.15)
                setPeriod(p, reset: true)
            }
        }

        totalCalls += 1
    }

    func setPeriod(_ p: WallDuration, reset: Bool = false) {
        timer?.setPeriod(period: p, reset: reset)
        expectedPeriod = p
    }

    func pretendWork(_ t: Double) {
        var r = Rate(duration: RosDuration(seconds: t))
        r.sleep()
    }
}



class TimerTest: RosTest {
    
    func testSingleSteadyTimeCallback() {
        let ros = Ros(master: host)
        let n = ros.createNode()

        let helper = SteadyTimerHelper(node: n, 0.01)
        let d = WallDuration(milliseconds: 1)
        for _ in 0..<1000 {
            ros.spinOnce()
            d.sleep()
        }

        XCTAssertFalse(helper.failed)
        XCTAssertGreaterThan(helper.totalCalls, 99)
    }

    func testMultipleSteadyTimeCallbacks() {
        let ros = Ros(master: host)
        XCTAssert(Time.useSimTime.load(ordering: .relaxed))

        let n = ros.createNode()
        XCTAssertFalse(Time.useSimTime.load(ordering: .relaxed))
        let helpers = (1...10).map {
            SteadyTimerHelper(node: n, Double($0)*0.1)
        }
        let start = WallTime.now
        let d = WallDuration(milliseconds: 10)
        for _ in 1...1000 {
            ros.spinOnce()
            d.sleep()
        }
        let end = WallTime.now
        let dur = (end - start).toSec()

        for h in helpers {
            XCTAssertFalse(h.failed)
            let expectedCount = Int(dur / h.expectedPeriod.toSec())
            XCTAssertEqual(h.totalCalls, expectedCount)
        }
    }


    func testSimClock() {
        let ros = Ros(master: host)
        ros.param.set(key: "/use_sim_time", value: true)
        let n = ros.createNode()
        XCTAssert(Time.useSimTime.load(ordering: .relaxed))
        let pub = n.advertise(topic: "/clock", message: rosgraph_msgs.Clock.self)!
        let start = Time.now
        
        XCTAssertTrue(start.isZero)
        
        pub.publish(message: rosgraph_msgs.Clock(clock: Time(seconds: 42)))
        
        ros.spinOnce()
        
        XCTAssertEqual(Time.now.toSec(), 42.0)
        ros.param.set(key: "/use_sim_time", value: false)

    }
    
    func testClockTimeValid() {
        Time.setNow(Time())
        XCTAssertFalse(Time.isValid)
        Time.setNow(Time.min)
        XCTAssertTrue(Time.isValid)
    }
    
    func testClockWaitForValid() {
        Time.setNow(Time())
        XCTAssert(Time.useSimTime.load(ordering: .relaxed))

        // test timeout
        
        let start = WallTime.now
        XCTAssertFalse(Time.waitForValid(timeout: WallDuration(milliseconds: 1000)))
        let end = WallTime.now
        XCTAssertGreaterThan(end-start, WallDuration(milliseconds: 1000))
        
        let done = ManagedAtomic(false)
        
        DispatchQueue(label: "waitThread").async {
            _ = Time.waitForValid()
            done.store(true, ordering: .relaxed)
        }
        
        WallDuration(milliseconds: 1000).sleep()
        XCTAssertFalse(done.load(ordering: .relaxed))
        
        Time.setNow(Time.min)
        
        while !done.load(ordering: .relaxed) {
            WallDuration(milliseconds: 1000).sleep()
        }
        
    }
    
    func testClockSleepFromZero() {
        Time.initialize()
        Time.setNow(Time())
        XCTAssert(Time.useSimTime.load(ordering: .relaxed))
        let done = ManagedAtomic(false)
        
        DispatchQueue(label: "sleep").async {
        	let ok = RosDuration(milliseconds: 1000).sleep()
            if !ok {
                print("!OK")
            }
            done.store(true, ordering: .relaxed)
        }
        
        WallDuration(milliseconds: 1000).sleep()
        let start = WallTime.now
        Time.setNow(Time(nanosec: start.nanoseconds))
        while !done.load(ordering: .relaxed) {
            WallDuration(milliseconds: 1).sleep()
            let now = WallTime.now
            Time.setNow(Time(nanosec: now.nanoseconds))
        }
        let end = WallTime.now
        XCTAssertGreaterThan(end-start, WallDuration(milliseconds: 1000))
    }
  
}
