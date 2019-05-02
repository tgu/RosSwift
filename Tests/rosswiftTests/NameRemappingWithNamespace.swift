//
//  NameRemappingWithNamespace.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-10-31.
//

import XCTest
@testable import RosSwift

class NameRemappingWithNamespace: XCTestCase {

    static var allTests = [
        ("testParameterRemapping",testParameterRemapping)
    ]

    override func setUp() {
    }

    override func tearDown() {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }

    func testParameterRemapping() {
        var args = CommandLine.arguments + ["/a/test_full:=/b/test_full",
                                            "/a/test_local:=test_local2",
                                            "test_relative:=/b/test_relative",
                                            "__ns:=a"]

        let ros = Ros(argv: &args, name: "testParameterRemapping")
        ros.param.set(key: "/b/test_full", value: "asdf")
        ros.param.set(key: "/a/test_local2", value: "asdf")
        ros.param.set(key: "/b/test_relative", value: "asdf")

        var param = ""

        XCTAssertEqual(ros.resolve(name: "test_full"), "/b/test_full")
        XCTAssert(ros.param.get("test_full", &param))
        XCTAssertEqual(ros.resolve(name: "/a/test_full"), "/b/test_full")
        XCTAssert(ros.param.get("/a/test_full", &param))

        XCTAssertEqual(ros.resolve(name: "test_local"), "/a/test_local2")
        XCTAssert(ros.param.get("test_local", &param))
        XCTAssertEqual(ros.resolve(name: "/a/test_local"), "/a/test_local2")
        XCTAssert(ros.param.get("/a/test_local", &param))

        XCTAssertEqual(ros.resolve(name: "test_relative"), "/b/test_relative")
        XCTAssert(ros.param.get("test_relative", &param))
        XCTAssertEqual(ros.resolve(name: "/a/test_relative"), "/b/test_relative")
        XCTAssert(ros.param.get("/a/test_relative", &param))

        XCTAssertNil(ros.resolve(name: "1244"))
        XCTAssertEqual(ros.resolve(name: "åäö"), "/a/åäö")

        let node_name = ros.name
        XCTAssertEqual(node_name, "/a/testParameterRemapping")

    }

}
