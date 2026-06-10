//
//  serviceTests.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-10-24.
//

import Testing
@testable import RosSwift
@testable import StdMsgs
@testable import BinaryCoder
import rosmaster
import RosNetwork
import Atomics

@Suite("Service tests", .serialized)
class ServiceTests {
    let master: rosmaster.Master
    var remap: [String: String] {
        ["__master": master.address]
    }
    var host: String {
        master.host
    }
    var port: Int {
        master.port
    }

    init() async throws {
        let network = RosNetwork(remappings: [:])
        master = rosmaster.Master(host: network.gHost, port: 0, advertise: false)
        _ = try await master.start().get()
    }

    deinit {
        _ = master.stop()
    }

    @Test func callService() async throws {
        @Sendable func srvCallback(req: TestStringString.Request) -> TestStringString.Response? {
            return TestStringString.Response("A" + req.data)
        }

        let rosProvider = try Ros(name: "testCallServiceProvider", remappings: remap)
        let serviceNode = await rosProvider.createNode()
        let serv = try #require(await serviceNode.advertise(service: "service_adv", srvFunc: srvCallback))

        let ros = try Ros(master: host, port: port)
        let node = await ros.createNode()

        var req = TestStringString.Request()
        var res = TestStringString.Response()
        req.data = "case_FLIP"

        let res1 = await Service.waitForService(ros: ros, serviceName: "/service_adv")
        #expect(res1)
        let firstCallSucceeded = await Service.call(node: node, serviceName: "/service_adv", req: req, response: &res)
        if firstCallSucceeded {
            print(res)
        }
        let secondCallSucceeded = await Service.call(node: node, serviceName: "/service_adv", req: req, response: &res)
        #expect(secondCallSucceeded)
        #expect(res.data == "Acase_FLIP")

        serv.shutdown()  // Just in case the ARC throws us away

        await ros.shutdownAsync()
        await rosProvider.shutdownAsync()
    }

    @Test func callEcho() async throws {
        try await withRos(master: host, port: port) { ros in
            let node = await ros.createNode()

            var req = TestStringString.Request()
            var res = TestStringString.Response()
            req.data = "case_FLIP"

            let s = await Service.waitForService(ros: ros, serviceName: "/echo", timeout: 10)
            #expect(!s)

            let srv2 = await TestStringString.advertise(service: "/echo", node: node) {
                let response = $0.data.uppercased()
                return .init(response)
            }

            extendLifetime(srv2)

            let s2 = await Service.waitForService(ros: ros, serviceName: "/echo", timeout: 10)
            #expect(s2)
            let message = await Service.call(node: node, serviceName: "/echo", req: req, response: &res)
            #expect(message)
            if message {
                #expect(res.data == req.data.uppercased())
            }
        }
    }

    @Test func callInternalService() async throws {
        try await withRos(master: host, port: port) { ros in
            @Sendable func serviceCallback(req: TestStringString.Request) -> TestStringString.Response? {
                return TestStringString.Response("test")
            }

            let n = await ros.createNode()

            let srv1 = await n.advertise(service: "/test_srv", srvFunc: serviceCallback)
            #expect(srv1 != nil)
            let res = await Service.call(node: n, name: "/test_srv", request: TestStringString.Request())
            #expect(res?.data == "test")
        }
    }

    @Test func trackedObjectWithServiceCallback() async throws {
        try await withRos(master: host, port: port) { ros in
            @Sendable func serviceCallback(req: TestStringString.Request) -> TestStringString.Response? {
                return TestStringString.Response("test")
            }

            final class Trackable: TrackableObject {
                let s = 0
            }

            var tracked: Trackable? = Trackable()

            let n = await ros.createNode()

            let srv1 = await n.advertise(service: "/test_srv", srvFunc: serviceCallback, tracked_object: tracked)
            #expect(srv1 != nil)
            let res1 = await Service.call(node: n, name: "/test_srv", request: TestStringString.Request())
            #expect(res1?.data == "test")

            tracked = nil

            let res2 = await Service.call(node: n, name: "/test_srv", request: TestStringString.Request())
            #expect(res2 == nil)
        }
    }

    @Test func serviceAdvCopy() async throws {
        let calls = ManagedAtomic(0)
        @Sendable func serviceCallback(req: TestStringString.Request) -> TestStringString.Response? {
            calls.wrappingIncrement(ordering: .relaxed)
            return TestStringString.Response("test")
        }

        let ros = try Ros(master: host, port: port)
        let node = await ros.createNode()
        let t = TestStringString.Request()

        let ros2 = try Ros(name: "testServiceAdvCopyCaller", remappings: remap)
        let node2 = await ros2.createNode()

        do {
            let srv1 = await node.advertise(service: "/test_srv_23", srvFunc: serviceCallback)
            try await Task.sleep(for: .seconds(4))
            let firstRes = await Service.call(node: node2, name: "/test_srv_23", request: t)
            #expect(firstRes != nil)
            do {
                let srv2 = srv1
                do {
                    let srv3 = srv2
                    #expect(srv3 === srv2)
                    let res = await Service.call(node: node2, name: "/test_srv_23", request: t)
                    #expect(res?.data == "test")
                }
                #expect(srv2 === srv1)
                let res2 = await Service.call(node: node2, name: "/test_srv_23", request: t)
                #expect(res2?.data == "test")
            }
            let res3 = await Service.call(node: node2, name: "/test_srv_23", request: t)
            #expect(res3?.data == "test")
        }
        try await Task.sleep(for: .seconds(1))
        let resAfterShutdown = await Service.call(node: node2, name: "/test_srv_23", request: t)
        #expect(resAfterShutdown == nil)

        #expect(calls.load(ordering: .relaxed) == 4)
        #expect(node.isOK)

        await ros.shutdownAsync()
        await ros2.shutdownAsync()
    }

    @Test func serviceAdvMultiple() async throws {
        try await withRos(master: host, port: port) { ros in
            let c1 = ManagedAtomic(0)
            let c2 = ManagedAtomic(0)
            @Sendable func serviceCallback1(req: TestStringString.Request) -> TestStringString.Response? {
                c1.wrappingIncrement(ordering: .relaxed)
                return TestStringString.Response("test")
            }
            @Sendable func serviceCallback2(req: TestStringString.Request) -> TestStringString.Response? {
                c2.wrappingIncrement(ordering: .relaxed)
                return TestStringString.Response("test")
            }

            let n = await ros.createNode()

            let srv = await n.advertise(service: "/test_srv_19", srvFunc: serviceCallback1)
            let srv2 = await n.advertise(service: "/test_srv_19", srvFunc: serviceCallback2)
            #expect(srv != nil)
            #expect(srv2 == nil)
            let t = TestStringString.Request()
            let resMulti = await Service.call(node: n, name: "/test_srv_19", request: t)
            #expect(resMulti != nil)
            #expect(c1.load(ordering: .relaxed) == 1)
            #expect(c2.load(ordering: .relaxed) == 0)
        }
    }

    @Test func callSrvMultipleTimes() async throws {
        try await withRos(master: host, port: port) { ros in
            let count = ManagedAtomic(0)
            @Sendable func srvCallback(req: TestStringString.Request) -> TestStringString.Response? {
                count.wrappingIncrement(ordering: .relaxed)
                return TestStringString.Response("A\(count.load(ordering: .relaxed))")
            }

            let node = await ros.createNode()
            let serv = await node.advertise(service: "/service_adv2", srvFunc: srvCallback)
            #expect(serv != nil)
            let req = TestStringString.Request("case_FLIP")
            var res = TestStringString.Response()

            let otherNode = await ros.createNode()

            for i in 0..<100 {
                let ok = await Service.call(node: otherNode, serviceName: "service_adv2", req: req, response: &res)
                #expect(ok)
                #expect(res.data == "A\(i + 1)")
            }
            #expect(count.load(ordering: .relaxed) == 100)
        }
    }
}
