//
//  serviceTests.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-10-24.
//

import XCTest
@testable import RosSwift
@testable import StdMsgs
@testable import BinaryCoder


class serviceTests: XCTestCase {

    static var allTests = [
        ("testCallService",testCallService),
        ("testCallEcho",testCallEcho),
        ("testCallInternalService",testCallInternalService),
        ("testServiceAdvCopy", testServiceAdvCopy),
        ("testServiceAdvMultiple",testServiceAdvMultiple),
        ("testCallSrvMultipleTimes",testCallSrvMultipleTimes)
    ]

    override func setUp() {
    }

    override func tearDown() {
    }

    func testCallService() {
        func srvCallback(req: TestStringString.Request) -> TestStringString.Response? {
            return TestStringString.Response("A")
        }

        let ros = Ros(name: "testCallService")
        let node = ros.createNode()
        guard let serv = node.advertise(service: "service_adv", srvFunc: srvCallback) else {
            XCTFail()
            return
        }

        var req = TestStringString.Request()
        var res = TestStringString.Response()
        req.data = "case_FLIP"

        XCTAssert(Service.waitForService(ros: ros, serviceName: "/service_adv"))
        if Service.call(node: node, serviceName: "/service_adv", req: req, response: &res) {
            print(res)
        }
        XCTAssert(Service.call(node: node, serviceName: "/service_adv", req: req, response: &res))
        XCTAssertEqual(res.data, "A")

        serv.shutdown()  // Just in case the ARC throws us away
    }

    func testCallEcho() {
        let ros = Ros(name: "testCallEcho")
        let node = ros.createNode()

        var req = TestStringString.Request()
        var res = TestStringString.Response()
        req.data = "case_FLIP"

        if Service.waitForService(ros: ros, serviceName: "/echo", timeout: 10 )  {
            let message = Service.call(node: node, serviceName: "/echo", req: req, response: &res)
            XCTAssert(message)
            if message {
                XCTAssertEqual(res.data,req.data)
            }
        }
    }

    func testCallInternalService() {
        func serviceCallback(req : TestStringString.Request) -> TestStringString.Response? {
            return TestStringString.Response("test")
        }


        let ros = Ros(name: "testCallInternalService")
        let n = ros.createNode()
        var t = TestStringString()

        let srv1 = n.advertise(service: "/test_srv", srvFunc: serviceCallback)
        XCTAssertNotNil(srv1)
        XCTAssert(Service.call(node: n, name: "/test_srv", service: &t))
        XCTAssertEqual(t.response.data, "test")


    }

    func testServiceAdvCopy()  {
        var calls = 0
        func serviceCallback(req : TestStringString.Request) -> TestStringString.Response? {
            calls += 1
            return TestStringString.Response("test")
        }

        let ros = Ros(name: "testServiceAdvCopy")
        let node = ros.createNode()
        var t = TestStringString()

        let ros2 = Ros(name: "testServiceAdvCopyCaller")
        let node2 = ros2.createNode()

        do {
            let srv1 = node.advertise(service: "/test_srv_23", srvFunc: serviceCallback)
            sleep(4)
            XCTAssert(Service.call(node: node2, name: "/test_srv_23", service: &t))
            do {
                let srv2 = srv1
                do {
                    let srv3 = srv2
                    XCTAssert(srv3 === srv2)
                    t.response.data = ""
                    XCTAssert(Service.call(node: node2, name: "/test_srv_23", service: &t))
                    XCTAssertEqual(t.response.data, "test")
                }
                XCTAssert(srv2 === srv1);
                t.response.data = ""
                XCTAssert(Service.call(node: node2, name: "/test_srv_23", service: &t))
                XCTAssertEqual(t.response.data, "test")
            }
            t.response.data = ""
            XCTAssert(Service.call(node: node2, name: "/test_srv_23", service: &t))
            XCTAssertEqual(t.response.data, "test")
        }
        sleep(1)
        XCTAssertFalse(Service.call(node: node2, name: "/test_srv_23", service: &t))

        XCTAssertEqual(calls, 4)
        print("\(node.isOK)")

    }






    func testServiceAdvMultiple()  {
        var c1 = 0
        var c2 = 0
        func serviceCallback1(req : TestStringString.Request) -> TestStringString.Response? {
            c1 += 1
            return TestStringString.Response("test")
        }
        func serviceCallback2(req : TestStringString.Request) -> TestStringString.Response? {
            c2 += 1
            return TestStringString.Response("test")
        }

        let ros = Ros(name: "testServiceAdvMultiple")
        let n = ros.createNode()

        let srv = n.advertise(service: "/test_srv_19", srvFunc: serviceCallback1)
        let srv2 = n.advertise(service: "/test_srv_19", srvFunc: serviceCallback2)
        XCTAssert(srv != nil)
        XCTAssertNil(srv2)
        var t = TestStringString()
        XCTAssert(Service.call(node: n, name: "/test_srv_19", service: &t))
        XCTAssertEqual(c1, 1)
        XCTAssertEqual(c2, 0)
    }



    func testCallSrvMultipleTimes() {

        var count = 0
        func srvCallback(req: TestStringString.Request) -> TestStringString.Response? {
            count += 1
            return TestStringString.Response("A\(count)")
        }


        let ros = Ros(name: "testCallSrvMultipleTimes")
        let node = ros.createNode()
        let serv = node.advertise(service: "/service_adv2", srvFunc: srvCallback)
        XCTAssertNotNil(serv)
        var req = TestStringString.Request()
        var res = TestStringString.Response()
        req.data = "case_FLIP"

        for i in 0..<100 {
            XCTAssert(Service.call(node: node, serviceName: "service_adv2", req: req, response: &res))
            XCTAssertEqual(res.data, "A\(i+1)")
        }
        XCTAssertEqual(count, 100)
    }



}
