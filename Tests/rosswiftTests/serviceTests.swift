//
//  serviceTests.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-10-24.
//

import XCTest
@testable import RosSwift
@testable import XMLRPCSerialization
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
        // Put setup code here. This method is called before the invocation of each test method in the class.
        Ros.initialize(argv: &CommandLine.arguments, name: "serviceTests")

    }

    override func tearDown() {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
        Ros.shutdown()
    }

    func srvCallback(req: TestStringString.Request) -> TestStringString.Response? {
        return TestStringString.Response("A")
    }

    func testCallService() {

        let node = Ros.NodeHandle()
        guard let serv = node.advertiseService(service: "/service_adv", srv_func: srvCallback) else {
            XCTFail()
            return
        }

        var req = TestStringString.Request()
        var res = TestStringString.Response()
        req.data = "case_FLIP"

        XCTAssert(service.waitForService(service_name: "/service_adv"))
        if service.call(service_name: "/service_adv", req: req, response: &res) {
            print(res)
        }
        XCTAssert(service.call(service_name: "/service_adv", req: req, response: &res))
        XCTAssertEqual(res.data, "A")

        serv.shutdown()  // Just in case the ARC throws us away
    }

    func testCallEcho() {
        let nh = Ros.NodeHandle()

        var req = TestStringString.Request()
        var res = TestStringString.Response()
        req.data = "case_FLIP"

        if service.waitForService(service_name: "/echo", timeout: 1000 )  {
            let message = service.call(service_name: "/echo", req: req, response: &res)
            XCTAssert(message)
            if message {
                XCTAssertEqual(res.data,req.data)
            }
        }
    }

    func serviceCallback(req : TestStringString.Request) -> TestStringString.Response? {
        return TestStringString.Response("test")
    }

    func testCallInternalService() {
        let n = Ros.NodeHandle()
        var t = TestStringString()

        let srv1 = n.advertiseService(service: "/test_srv", srv_func: serviceCallback)
        XCTAssert(service.call(name: "/test_srv", service: &t))
        XCTAssertEqual(t.response.data, "test")


    }

    func testServiceAdvCopy()  {
        let n = Ros.NodeHandle()
        var t = TestStringString()

        do {
            let srv1 = n.advertiseService(service: "/test_srv_23", srv_func: serviceCallback)
            sleep(4)
            XCTAssert(service.call(name: "/test_srv_23", service: &t))
            do {
                let srv2 = srv1
                do {
                    let srv3 = srv2
                    XCTAssert(srv3 === srv2)
                    t.response.data = ""
                    XCTAssert(service.call(name: "/test_srv_23", service: &t))
                    XCTAssertEqual(t.response.data, "test")
                }
                XCTAssert(srv2 === srv1);
                t.response.data = ""
                XCTAssert(service.call(name: "/test_srv_23", service: &t))
                XCTAssertEqual(t.response.data, "test")
            }
            t.response.data = ""
            XCTAssert(service.call(name: "/test_srv_23", service: &t))
            XCTAssertEqual(t.response.data, "test")
        }
        XCTAssertFalse(service.call(name: "/test_srv_23", service: &t))

        print("\(n.ok)")

    }






    func testServiceAdvMultiple()  {
        let n = Ros.NodeHandle()

        let srv = n.advertiseService(service: "/test_srv_19", srv_func: serviceCallback)
        let srv2 = n.advertiseService(service: "/test_srv_19", srv_func: serviceCallback)
        XCTAssert(srv != nil)
        XCTAssertNil(srv2)

    }



    func testCallSrvMultipleTimes() {

        let node = Ros.NodeHandle()
        guard let serv = node.advertiseService(service: "/service_adv2", srv_func: srvCallback) else {
            XCTFail()
            return
        }

        var req = TestStringString.Request()
        var res = TestStringString.Response()
        req.data = "case_FLIP"

//        self.measure {
            for i in 0..<10 {
                XCTAssert(service.call(service_name: "service_adv2", req: req, response: &res))
            }
//        }
    }



}
