//
//  serializationTests.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-11-07.
//

import Testing
@testable import StdMsgs
@testable import RosSwift
@testable import RosTime
@testable import BinaryCoder
@testable import msgs
import rosmaster
import RosNetwork


extension sensor_msgs.Imu: Equatable {
    public static func == (lhs: sensor_msgs.Imu, rhs: sensor_msgs.Imu) -> Bool {
        return lhs.angular_velocity_covariance == rhs.angular_velocity_covariance
    }
}

extension shape_msgs.MeshTriangle: Equatable {
    public static func == (lhs: shape_msgs.MeshTriangle, rhs: shape_msgs.MeshTriangle) -> Bool {
        return lhs.vertex_indices == rhs.vertex_indices
    }
}

func serialize<T: BinaryCodable>(_ value: T) -> [UInt8] {
    let data = try! BinaryEncoder.encode(value)
    let buf = try! BinaryEncoder.encode(UInt32(data.count))
    return buf + data
}

func deserialize<T: BinaryCodable>(_ buffer: [UInt8]) -> T {
    let b = [UInt8](buffer.dropFirst(4))
    return try! BinaryDecoder.decode(T.self, data: b)
}

func serializeAndDeserialize<T: BinaryCodable>(_ ser_val: T) -> T {
    let buffer = serialize(ser_val)
    let m: T = deserialize(buffer)
    return m
}

@Suite("Serialization tests")
struct SerializationTests {

    @Test func primitive() {
        #expect(UInt8(5) == serializeAndDeserialize(UInt8(5)))
        #expect(UInt16(5) == serializeAndDeserialize(UInt16(5)))
        #expect(UInt32(5) == serializeAndDeserialize(UInt32(5)))
        #expect(UInt64(5) == serializeAndDeserialize(UInt64(5)))
        #expect(Int8(5) == serializeAndDeserialize(Int8(5)))
        #expect(Int16(5) == serializeAndDeserialize(Int16(5)))
        #expect(Int32(5) == serializeAndDeserialize(Int32(5)))
        #expect(Int64(5) == serializeAndDeserialize(Int64(5)))
        #expect(Int8(5) == serializeAndDeserialize(Int8(5)))
        #expect(Float(5) == serializeAndDeserialize(Float(5)))
        #expect(Float32(5) == serializeAndDeserialize(Float32(5)))
        #expect(Float64(5) == serializeAndDeserialize(Float64(5)))
        #expect(Double(5) == serializeAndDeserialize(Double(5)))
        #expect(Time(sec: 100, nsec: 234) == serializeAndDeserialize(Time(sec: 100, nsec: 234)))
        #expect(RosDuration(nanosec: 12234) == serializeAndDeserialize(RosDuration(nanosec: 12234)))
        #expect("string" == serializeAndDeserialize("string"))
        var str = "hello world"
        str.append("hello world22")
        str.append("hello world333")
        str.append("hello world4444")
        str.append("hello world55555")
        #expect(str == serializeAndDeserialize(str))
    }

    @Test func fixedArray() {
        var msg = sensor_msgs.Imu()
        msg.angular_velocity_covariance.array = [1, 2, 3, 4, 5, 6, 7, 8, 9]
        #expect(msg == serializeAndDeserialize(msg))
    }

    @Test func fixedUInt32Array() {
        let msg = shape_msgs.MeshTriangle(vertex_indices: [99, 23, 1245])
        #expect(msg == serializeAndDeserialize(msg))
    }
}

@Suite("Serialization tests with ros master", .serialized)
class SerializationWithMasterTests {
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

    @Sendable static func callbackB(_ val: Bool) {}
    @Sendable static func callbackI(_ val: Int32) {}
    @Sendable static func callbackD(_ val: Double) {}

    @Test func builtinTypes() async throws {
        try await withRos(master: host, port: port) { ros in
        let n = await ros.createNode()
        let p1 = await n.advertise(topic: "test_bool", message: Bool.self)
        #expect(p1 != nil)
        let p2 = await n.advertise(topic: "test_int", message: Int32.self)
        #expect(p2 != nil)
        let p3 = await n.advertise(topic: "test_double", message: Double.self)
        #expect(p3 != nil)

        let s1 = await n.subscribe(topic: "test_bool", queueSize: 1, callback: SerializationWithMasterTests.callbackB)
        let s2 = await n.subscribe(topic: "test_int", queueSize: 1, callback: SerializationWithMasterTests.callbackI)
        let s3 = await n.subscribe(topic: "test_double", queueSize: 1, callback: SerializationWithMasterTests.callbackD)
        #expect(s1?.publisherCount == 1)
        #expect(s2?.publisherCount == 1)
        #expect(s3?.publisherCount == 1)
        #expect(p1 != nil)
        #expect(p2 != nil)
        #expect(s1 != nil)
        #expect(s2 != nil)
        }
    }
}
