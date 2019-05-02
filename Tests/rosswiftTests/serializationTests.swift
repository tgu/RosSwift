//
//  serializationTests.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-11-07.
//

import XCTest
@testable import StdMsgs
@testable import RosSwift
@testable import RosTime
@testable import BinaryCoder


func serializeAndDeserialize<T : BinaryCodable>(_ ser_val: T) -> T {
    let buffer = serialize(ser_val)
    let m : T = deserialize(buffer)
    return m
}

class serializationTests: XCTestCase {

    static var allTests = [
        ("testPrimitive",testPrimitive),
        ("testBuiltinTypes",testBuiltinTypes)
    ]


    override func setUp() {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }

    override func tearDown() {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }

    func testPrimitive() {
        XCTAssertEqual(UInt8(5), serializeAndDeserialize(UInt8(5)))
        XCTAssertEqual(UInt16(5), serializeAndDeserialize(UInt16(5)))
        XCTAssertEqual(UInt32(5), serializeAndDeserialize(UInt32(5)))
        XCTAssertEqual(UInt64(5), serializeAndDeserialize(UInt64(5)))
        XCTAssertEqual(Int8(5), serializeAndDeserialize(Int8(5)))
        XCTAssertEqual(Int16(5), serializeAndDeserialize(Int16(5)))
        XCTAssertEqual(Int32(5), serializeAndDeserialize(Int32(5)))
        XCTAssertEqual(Int64(5), serializeAndDeserialize(Int64(5)))
        XCTAssertEqual(Int8(5), serializeAndDeserialize(Int8(5)))
        XCTAssertEqual(Float(5), serializeAndDeserialize(Float(5)))
        XCTAssertEqual(Float32(5), serializeAndDeserialize(Float32(5)))
        XCTAssertEqual(Float64(5), serializeAndDeserialize(Float64(5)))
        XCTAssertEqual(Double(5), serializeAndDeserialize(Double(5)))
        XCTAssertEqual(Time(sec: 100, nsec: 234), serializeAndDeserialize(Time(sec: 100, nsec: 234)))
        XCTAssertEqual(Duration(nanosec: 12234), serializeAndDeserialize(Duration(nanosec: 12234)))
        XCTAssertEqual("string", serializeAndDeserialize("string"))
        var str = "hello world"
        str.append("hello world22")
        str.append("hello world333")
        str.append("hello world4444")
        str.append("hello world55555")
        XCTAssertEqual(str, serializeAndDeserialize(str))

    }


    func callbackB(_ val: Bool) {

    }

    func callbackI(_ val: Int32) {

    }

    func callbackD(_ val: Double) {

    }

    func testBuiltinTypes() {
        let ros = Ros(argv: &CommandLine.arguments, name: "builtinTests")
        let n = ros.createNode()
        let p1 = n.advertise(topic: "test_bool", message: Bool.self)
        XCTAssertNotNil(p1)
        let p2 = n.advertise(topic: "test_int", message: Int32.self)
        XCTAssertNotNil(p2)
        let p3 = n.advertise(topic: "test_double", message: Double.self)
        XCTAssertNotNil(p3)



        let s1 = n.subscribe(topic: "test_bool", queueSize: 1, callback: callbackB)
        let s2 = n.subscribe(topic: "test_int", queueSize: 1, callback: callbackI)
        let s3 = n.subscribe(topic: "test_double", queueSize: 1, callback: callbackD)
        XCTAssertEqual(s1!.publisherCount,1)
        XCTAssertEqual(s2!.publisherCount,1)
        XCTAssertEqual(s3!.publisherCount,1)
        XCTAssertNotNil(p1)
        XCTAssertNotNil(p2)
        XCTAssertNotNil(s1)
        XCTAssertNotNil(s2)
    }

}
