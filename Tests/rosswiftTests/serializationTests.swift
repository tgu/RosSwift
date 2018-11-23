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
        XCTAssertEqual(RosTime.Time(sec: 100, nsec: 234), serializeAndDeserialize(RosTime.Time(sec: 100, nsec: 234)))
        XCTAssertEqual(RosTime.Duration(nanosec: 12234), serializeAndDeserialize(RosTime.Duration(nanosec: 12234)))
        XCTAssertEqual("string", serializeAndDeserialize("string"))
        var str = "hello world"
        str.append("hello world22")
        str.append("hello world333")
        str.append("hello world4444")
        str.append("hello world55555")
        XCTAssertEqual(str, serializeAndDeserialize(str))

    }


    func callback<T : Message>(_ val: T) {

    }

    func testBuiltinTypes() {
        Ros.initialize(argv: &CommandLine.arguments, name: "builtinTests")
        let n = Ros.NodeHandle()
        let p1 = n.advertise(topic: "test_bool", message: Bool.self)
        let p2 = n.advertise(topic: "test_int", message: Int32.self)
        let p3 = n.advertise(topic: "test_double", message: Double.self)



        let s1 = n.subscribe(Bool.self, topic: "test_bool", queueSize: 1, callback: callback)
        let s2 = n.subscribe(Int32.self, topic: "test_int", queueSize: 1, callback: callback)
        let s3 = n.subscribe(Double.self, topic: "test_double", queueSize: 1, callback: callback)
        XCTAssertEqual(s1!.getNumPublishers(),1)
        XCTAssertEqual(s2!.getNumPublishers(),1)
        XCTAssertEqual(s3!.getNumPublishers(),1)
        XCTAssertNotNil(p1)
        XCTAssertNotNil(p2)
        XCTAssertNotNil(s1)
        XCTAssertNotNil(s2)
    }

}
