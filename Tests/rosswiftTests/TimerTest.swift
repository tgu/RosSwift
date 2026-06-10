//
//  TimerTest.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2019-04-24.
//

import Testing
@testable import RosSwift
@testable import RosTime
@testable import msgs
import Atomics
import rosmaster
import RosNetwork
import AsyncAlgorithms

@Suite(.serialized)
class TimerTesting {
    let master: rosmaster.Master
    var remap: [String:String] {
        ["__master": master.address]
    }
    var host: String {
        master.host
    }
    var port: Int {
        master.port
    }

    init() throws {
        let network = RosNetwork(remappings: [:])
        master = rosmaster.Master(host: network.gHost, port: 0, advertise: false)
        _ = try master.start().wait()
    }

    deinit {
        _ = master.stop()
    }


    @Test func singleSteadyTimeCallback() async throws {
        let ros = try Ros(master: host, port: port)
        let n = await ros.createNode()

        let helper = await SteadyTimerHelper(node: n, 0.1)
        let d = WallDuration(milliseconds: 10)
        let start = ContinuousClock.now
        for _ in 0..<100 {
            await ros.spinOnce()
            await d.sleep()
        }

        let elapsed = ContinuousClock.now - start
        print("elapsed: \(elapsed)")

        #expect(helper.failed == false)
        #expect(helper.totalCalls > 10)
    }

    @Test func multipleSteadyTimeCallbacks() async throws {
        let ros = try Ros(master: host, port: port)
        // Note: `Time.useSimTime` is a process-global atomic flipped by
        // `ros.start()`. Its value isn't observable in a stable way when
        // suites run in parallel, so we don't assert on it here — this
        // test is about multiple steady-time callbacks, not sim-time state.
        let n = await ros.createNode()

        try await withThrowingTaskGroup(of: SteadyTimerHelper.self) { group in
            for i in 1...10 {
                group.addTask {
                    await SteadyTimerHelper(node: n, Double(i)*0.2)
                }
            }

            let start = WallTime.now
            let d = WallDuration(milliseconds: 1)
            for _ in 1...1000 {
                Task { await ros.spinOnce() }
                await d.sleep()
            }
            let end = WallTime.now
            let dur = (end - start).toSec()
            var count = 0
            for try await h in group {
                count += 1
                #expect(h.failed == false)
                let expectedCount = Int(dur / h.expectedPeriod.toSec())
                #expect(h.totalCalls == expectedCount, "for expected period: \(h.expectedPeriod.toSec() * 1000)")
            }
            #expect(count == 10)
        }
    }

    @Test func steadySetPeriod() async throws {
        let ros = try Ros(master: host, port: port)
        let n = await ros.createNode()
        let period = WallDuration(seconds: 0.5)
        let helper = await SteadyTimerHelper(node: n, period.toSec())
        let r = Rate(frequency: 100)

        // Let the callback occur once before getting started
        while helper.totalCalls < 1 {
            await ros.spinOnce()
            await r.sleep()
        }

        await helper.pretendWork(0.1)

        // outside callback, new period < old period, reset = false
        let p = WallDuration(seconds: 0.25)
        await helper.setPeriod(p)
        while helper.totalCalls < 2 {
            await ros.spinOnce()
            await r.sleep()
        }

        await helper.pretendWork(0.1)

        // outside callback, new period > old period, reset = false
        let p2 = WallDuration(seconds: 0.5)
        await helper.setPeriod(p2)
        while helper.totalCalls < 3 {
            await ros.spinOnce()
            await r.sleep()
        }

        await helper.pretendWork(0.1)

        // outside callback, new period < old period, reset = true
        let p3 = WallDuration(seconds: 0.25)
        await helper.setPeriod(p3, reset: true)
        while helper.totalCalls < 4 {
            await ros.spinOnce()
            await r.sleep()
        }

        await helper.pretendWork(0.1)

        // outside callback, new period > old period, reset = true

        let p4 = WallDuration(seconds: 0.5)
        await helper.setPeriod(p4, reset: true)
        while helper.totalCalls < 5 {
            await ros.spinOnce()
            await r.sleep()
        }

        // Test calling setPeriod inside callback
        helper.callsBeforeTestingPeriod = helper.totalCalls
        let total = helper.totalCalls + 5
        helper.testingPeriod = true
        while helper.totalCalls < total {
            await ros.spinOnce()
            await r.sleep()
        }

        helper.testingPeriod = false

        #expect(helper.failed == false, "Helper failed in setPeriod")
    }

    @Test func stopSteadyTimer() async throws {
        let ros = try Ros(master: host, port: port)
        let n = await ros.createNode()
        let helper = await SteadyTimerHelper(node: n, 0.001)

        for _ in 0..<1000 {
            await WallDuration(milliseconds: 1).sleep()
            await ros.spinOnce()
        }
        #expect(helper.totalCalls > 0)
        let lastCount = helper.totalCalls
        await helper.timer?.stop()

        for _ in 0..<1000 {
            await WallDuration(milliseconds: 1).sleep()
            await ros.spinOnce()
        }

        #expect(helper.totalCalls == lastCount)

    }

    @Test func steadyStopThenSpin() async throws {
        let ros = try Ros(master: host, port: port)
        let n = await ros.createNode()
        let g_steady_count = ManagedAtomic(0)

        let timer = await n.createSteadyTimer(period: WallDuration(milliseconds: 1)) { event in
            g_steady_count.wrappingIncrement(ordering: .relaxed)
        }

        await WallDuration(milliseconds: 100).sleep()
        await timer.stop()

        await ros.spinOnce()

        #expect(g_steady_count.load(ordering: .relaxed) == 0)
    }

    @Test func oneShotSteadyTimer() async throws {
        let ros = try Ros(master: host, port: port)
        let n = await ros.createNode()
        let helper = await SteadyTimerHelper(node: n, 0.001, oneshot: true)

        for _ in 0..<1000 {
            await WallDuration(milliseconds: 1).sleep()
            await ros.spinOnce()
        }

        #expect(helper.totalCalls == 1)
    }


    @Test func singleTimeCallback() async throws {
        let ros = try Ros(master: host, port: port)
        let n = await ros.createNode()

        var now = Time(sec: 1, nsec: 0)
        Time.setNow(now)

        let helper = await TimerHelper(node: n, period: RosDuration(sec: 0, nsec: 10_000_000))

        let d = RosDuration(sec: 0, nsec: 1_000_000)
        for _ in 0..<1000 {
            now += d
            Time.setNow(now)

            while await helper.timer?.hasPending() == .some(true) {
                await WallDuration(milliseconds: 1).sleep()
                await ros.spinOnce()
            }

        }

        #expect(helper.failed == false)

        #expect(helper.total_calls == 100)
    }

    @Test func singleTimeCallbackFromRate() async throws {
        let ros = try Ros(master: host, port: port)
        let n = await ros.createNode()

        var now = Time(sec: 1, nsec: 0)
        Time.setNow(now)

        let helper = await TimerHelper(node: n, rate: Rate(frequency: 100))

        let d = RosDuration(sec: 0, nsec: 1_000_000)
        for _ in 0..<1000 {
            now += d
            Time.setNow(now)

            while await helper.timer?.hasPending() == .some(true) {
                await WallDuration(seconds: 0.00025).sleep()
                await ros.spinOnce()
            }

        }

        #expect(helper.failed == false)

        #expect(helper.total_calls == 100)
    }


    @Test func oneShotTimer() async throws {
        let ros = try Ros(master: host, port: port)
        let n = await ros.createNode()

        var now = Time(sec: 1, nsec: 0)
        Time.setNow(now)

        let helper = await TimerHelper(node: n, period: RosDuration(sec: 0, nsec: 10_000_000), oneShot: true)

        let d = RosDuration(sec: 0, nsec: 1_000_000)
        for _ in 0..<1000 {
            now += d
            Time.setNow(now)

            while await helper.timer?.hasPending() == .some(true) {
                await WallDuration(milliseconds: 1).sleep()
                await ros.spinOnce()
            }

        }

        #expect(helper.failed == false)

        #expect(helper.total_calls == 1)
    }

    @Test func singleTimeCallbackLargeTimestep() async throws {
        let ros = try Ros(master: host, port: port)
        let n = await ros.createNode()

        var now = Time(sec: 1, nsec: 0)
        Time.setNow(now)

        let helper = await TimerHelper(node: n, period: RosDuration(sec: 0, nsec: 10_000_000))

        let d = RosDuration(sec: 0, nsec: 100_000_000)
        for _ in 0..<100 {
            now += d
            Time.setNow(now)

            while await helper.timer?.hasPending() == .some(true) {
                await WallDuration(milliseconds: 1).sleep()
                await ros.spinOnce()
            }

        }

        #expect(helper.failed == false)
        #expect(helper.total_calls == 200)
    }

    @Test func multipleTimeCallback() async throws {
        let ros = try Ros(master: host, port: port)
        let n = await ros.createNode()

        var now = Time(sec: 1, nsec: 0)
        Time.setNow(now)

        let count = 100
        var helpers = [TimerHelper]()

       for i in 1...count {
            let h = await TimerHelper(node: n, period: RosDuration(seconds: 0.01 * Double(i)))
           helpers.append(h)
        }

        let d = RosDuration(milliseconds: 1)
        let spin_count = 1000
        for _ in 0..<spin_count {
            now += d
            Time.setNow(now)

            var pending = false

            repeat {
                pending = false
                for i in 0..<count {
                    if pending { break }
                    let pend = await helpers[i].timer?.hasPending() == .init(true)
                    pending = pending || pend
                }

                await WallDuration(milliseconds: 1).sleep()
                await ros.spinOnce()
            } while pending

        }

        for i in 0..<count {
            #expect(helpers[i].failed == false)

            let expected_count = Int((Double(spin_count) * d.toSec()) / helpers[i].expected_period.toSec())
            #expect(helpers[i].total_calls == expected_count)
        }

    }

    static let g_count = ManagedAtomic(0)


    /// Verifies that a timer associated with a tracked object stops invoking the callback
    /// once the tracked object is deallocated.
    ///
    /// Expectations:
    /// - The counter is greater than zero before deallocation (callbacks occurred).
    /// - The counter does not increase after deallocation (callbacks stop).
    @Test func trackedObject() async throws {
        final class Tracked: TrackableObject {
            init() {
                TimerTesting.g_count.store(0, ordering: .relaxed)
            }

            deinit {
                ROS_DEBUG("Tracked deinit")
            }

            func callback(_ event: TimerEvent) {
                TimerTesting.g_count.wrappingIncrement(ordering: .relaxed)
            }
        }

        let ros = try Ros(master: host, port: port)
        let n = await ros.createNode()
        var now = Time(sec: 1, nsec: 0)
        Time.setNow(now)

        var tracked: Tracked? = Tracked()
        let timer = await n.createTimer(period: RosDuration(milliseconds: 1), trackedObject: tracked, callback: tracked!.callback)

        now += RosDuration(milliseconds: 100)
        Time.setNow(now)

        while await timer.hasPending() {
            await WallDuration(milliseconds: 1).sleep()
            await ros.spinOnce()
        }

        let lastCount = TimerTesting.g_count.load(ordering: .relaxed)

        #expect(lastCount > 0)

        tracked = nil

        now += RosDuration(milliseconds: 100)
        Time.setNow(now)

        while await timer.hasPending() {
            await WallDuration(milliseconds: 1).sleep()
            await ros.spinOnce()
        }

        withKnownIssue("callback is holding a reference to the tracked object, causing extra callbacks") {
            #expect(TimerTesting.g_count.load(ordering: .relaxed) == lastCount)
        }

    }


    @Test func simClock() async throws {
        let ros = try Ros(master: host, port: port)
        await ros.param.set(key: "/use_sim_time", value: true)
        let n = await ros.createNode()
        #expect(Time.useSimTime.load(ordering: .relaxed) == true)
        let pub = await n.advertise(topic: "/clock", message: rosgraph_msgs.Clock.self)!
        let start = Time.now

        #expect(start.isZero)

        await pub.publish(message: rosgraph_msgs.Clock(clock: Time(seconds: 42)))

        await ros.spinOnce()

        #expect(Time.now.toSec() == 42.0)
        await ros.param.set(key: "/use_sim_time", value: false)

    }

    @Test func clockTimeValid() {
        Time.setNow(Time())
        #expect(Time.isValid == false)
        Time.setNow(Time.min)
        #expect(Time.isValid)
    }

    @Test func clockWaitForValid() async throws {
        Time.setNow(Time())
        #expect(Time.useSimTime.load(ordering: .relaxed) == true)

        // test timeout

        let start = WallTime.now
        let valid = await Time.waitForValid(timeout: WallDuration(milliseconds: 1000))
        #expect(valid == false)
        let end = WallTime.now
        #expect(end-start > WallDuration(milliseconds: 1000))

        let done = ManagedAtomic(false)

        Task.detached {
            _ = await Time.waitForValid()
            done.store(true, ordering: .relaxed)
        }

        await WallDuration(milliseconds: 1000).sleep()
        #expect(done.load(ordering: .relaxed) == false)

        Time.setNow(Time.min)

        while !done.load(ordering: .relaxed) {
            await WallDuration(milliseconds: 1000).sleep()
        }

    }

    @Test func clockSleepFromZero() async throws {
        Time.initialize()
        Time.setNow(Time())
        #expect(Time.useSimTime.load(ordering: .relaxed) == true)
        let done = ManagedAtomic(false)

        Task.detached {
            let ok = await RosDuration(milliseconds: 1000).sleep()
            if !ok {
                print("!OK")
            }
            done.store(true, ordering: .relaxed)
        }

        await WallDuration(milliseconds: 1000).sleep()
        let start = WallTime.now
        Time.setNow(Time(nanosec: start.nanoseconds))
        while !done.load(ordering: .relaxed) {
            await WallDuration(milliseconds: 1).sleep()
            let now = WallTime.now
            Time.setNow(Time(nanosec: now.nanoseconds))
        }
        let end = WallTime.now
        #expect(end-start > WallDuration(milliseconds: 1000))
    }
}



final class SteadyTimerHelper: @unchecked Sendable, TrackableObject {
    var expectedPeriod: WallDuration
    let oneshot: Bool
    var failed = false
    var totalCalls = 0
    var testingPeriod = false
    var callsBeforeTestingPeriod = 0
    var timer: SteadyTimer? = nil
    var lastCall = SteadyTime(nanosec: 0)

    init(node: NodeHandle, _ period: Double, oneshot: Bool = false) async {
        self.oneshot = oneshot
        let d = WallDuration(seconds: period)
        self.expectedPeriod = d
        let tim = await node.createSteadyTimer(period: d, oneshot: oneshot, trackedObject: self, callback: callback)
        self.timer = tim
    }

    @Sendable
    func callback(event: SteadyTimerEvent) async {
        let first = lastCall.isZero
        lastCall = event.currentExpired

        if !first {
            let time_error = event.currentExpired.toSec() - event.currentExpected.toSec()
            if time_error > 5.0 || time_error < -0.01 {
                ROS_ERROR("Call came at wrong time: \(time_error)")
                failed = true
            }
        }

        if testingPeriod {
            if totalCalls == callsBeforeTestingPeriod {
                let p = WallDuration(milliseconds: 500)
                await pretendWork(0.15)
                await setPeriod(p)
            } else if totalCalls == callsBeforeTestingPeriod + 1 {
                let p = WallDuration(milliseconds: 250)
                await pretendWork(0.15)
                await setPeriod(p)
            } else if totalCalls == callsBeforeTestingPeriod + 2 {
                let p = WallDuration(milliseconds: 500)
                await pretendWork(0.15)
                await setPeriod(p, reset: true)
            } else if totalCalls == callsBeforeTestingPeriod + 3 {
                let p = WallDuration(milliseconds: 250)
                await pretendWork(0.15)
                await setPeriod(p, reset: true)
            }
        }

        totalCalls += 1
    }

    func setPeriod(_ p: WallDuration, reset: Bool = false) async {
        await timer?.setPeriod(period: p, reset: reset)
        expectedPeriod = p
    }

    func pretendWork(_ t: Double) async {
        let r = Rate(duration: RosDuration(seconds: t))
        _ = await r.sleep()
    }
}

final class TimerHelper: @unchecked Sendable, TrackableObject {
    let failed: Bool
    let expected_period: RosDuration
    var timer: Timer? = nil
    var total_calls: Int

    init(node: NodeHandle, period: RosDuration, oneShot: Bool = false) async {
        self.expected_period = period
        self.failed = false
        self.total_calls = 0
        let tim = await node.createTimer(period: expected_period, oneshot: oneShot, trackedObject: self, callback: callback)
        self.timer = tim
    }

    init(node: NodeHandle, rate: Rate, oneShot: Bool = false) async {
        self.expected_period = rate.expectedCycleTime
        self.failed = false
        self.total_calls = 0
        let tim = await node.createTimer(period: rate.expectedCycleTime, oneshot: oneShot, trackedObject: self, callback: callback)
        self.timer = tim
    }


    @Sendable func callback(_: TimerEvent) {
        total_calls += 1
    }


}

