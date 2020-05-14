//
//  Message.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import BinaryCoder

public protocol Message: BinaryCodable {
    static var md5sum : String { get }
    static var datatype : String { get }
    static var definition: String { get }
}

public protocol MessageWithHeader: Message {
    var header: std_msgs.Header { get set }
}

public protocol ServiceMessage: Message {
    associatedtype ServiceType: ServiceProt
}

extension ServiceMessage {
    public static var srvMd5sum: String {
        return ServiceType.md5sum
    }

    public static var srvDatatype: String {
        return ServiceType.datatype
    }
}

public protocol ServiceRequestMessage: ServiceMessage {}

public protocol ServiceResponseMessage: ServiceMessage {}

public protocol ServiceProt {
    associatedtype Request: ServiceRequestMessage
    associatedtype Response: ServiceResponseMessage
    static var md5sum: String { get }
    static var datatype: String { get }
}

// Builtin native types

extension Bool: Message {
    public static let md5sum = std_msgs.bool.md5sum
    public static let datatype = std_msgs.bool.datatype
    public static let definition = std_msgs.bool.definition
}

extension Int8: Message {
    public static let md5sum = std_msgs.int8.md5sum
    public static let datatype = std_msgs.int8.datatype
    public static let definition = std_msgs.int8.definition
}

extension Int16: Message {
    public static let md5sum = std_msgs.int16.md5sum
    public static let datatype = std_msgs.int16.datatype
    public static let definition = std_msgs.int16.definition
}

extension Int32: Message {
    public static let md5sum = std_msgs.int32.md5sum
    public static let datatype = std_msgs.int32.datatype
    public static let definition = std_msgs.int32.definition
}

extension Int64: Message {
    public static let md5sum = std_msgs.int64.md5sum
    public static let datatype = std_msgs.int64.datatype
    public static let definition = std_msgs.int64.definition
}

extension UInt8: Message {
    public static let md5sum = std_msgs.uint8.md5sum
    public static let datatype = std_msgs.uint8.datatype
    public static let definition = std_msgs.uint8.definition
}

extension UInt16: Message {
    public static let md5sum = std_msgs.uint16.md5sum
    public static let datatype = std_msgs.uint16.datatype
    public static let definition = std_msgs.uint16.definition
}

extension UInt32: Message {
    public static let md5sum = std_msgs.uint32.md5sum
    public static let datatype = std_msgs.uint32.datatype
    public static let definition = std_msgs.uint32.definition
}

extension UInt64: Message {
    public static let md5sum = std_msgs.uint64.md5sum
    public static let datatype = std_msgs.uint64.datatype
    public static let definition = std_msgs.uint64.definition
}

extension Float32: Message {
    public static let md5sum = std_msgs.float32.md5sum
    public static let datatype = std_msgs.float32.datatype
    public static let definition = std_msgs.float32.definition
}

extension Float64: Message {
    public static let md5sum = std_msgs.float64.md5sum
    public static let datatype = std_msgs.float64.datatype
    public static let definition = std_msgs.float64.definition
}

extension String: Message {
    public static let md5sum = std_msgs.string.md5sum
    public static let datatype = std_msgs.string.datatype
    public static let definition = std_msgs.string.definition
}


// Ugly hack for fixed length arrays. The serialized message contains no length for
// fixed array. This is a workaround until Swift gets support for fixed array types

public protocol FixedLengthFloatArray: BinaryCodable {
    static var length: Int { get }
    var _array: [Float64] { get set }
}

public extension FixedLengthFloatArray {
    var array: [Float64] {
        get {
            return _array
        }
        set {
            precondition(newValue.count == Self.length)
            _array = newValue
        }
    }

    subscript(i: Int) -> Float64 {
        get {
            return _array[i]
        }
        set {
            _array[i] = newValue
        }
    }

    // Fixed arrays has no extra length code

    func binaryEncode(to encoder: BinaryEncoder) throws {
        precondition(_array.count == Self.length)
        for a in _array {
            try a.binaryEncode(to: encoder)
        }
    }
}

public struct FixedLengthFloat64Array4: FixedLengthFloatArray, Equatable {
    public static let length: Int = 4
    public var _array: [Float64]

    public init() {
        _array = [Float64](repeating: 0, count: 4)
    }

    public init(_ arr: [Float64]) {
        precondition(arr.count == 4)
        _array = arr
    }

    public init(fromBinary decoder: BinaryDecoder) throws {
        _array = try (0 ..< 4).map { _ in try Float64(from: decoder) }
    }

}



public struct FixedLengthFloat64Array9: FixedLengthFloatArray, Equatable {
    public static let length: Int = 9
    public var _array: [Float64]

    public init() {
        _array = [Float64](repeating: 0, count: 9)
    }

    public init(_ arr: [Float64]) {
        precondition(arr.count == 9)
        _array = arr
    }

    public init(fromBinary decoder: BinaryDecoder) throws {
        _array = try (0 ..< 9).map { _ in try Float64(from: decoder) }
    }

}

public struct FixedLengthFloat64Array12: FixedLengthFloatArray, Equatable {
    public static let length: Int = 12
    public var _array: [Float64]

    public init() {
        _array = [Float64](repeating: 0, count: 12)
    }

    public init(_ arr: [Float64]) {
        precondition(arr.count == 12)
        _array = arr
    }

    public init(fromBinary decoder: BinaryDecoder) throws {
        _array = try (0 ..< 12).map { _ in try Float64(from: decoder) }
    }

}

public struct FixedLengthFloat64Array15: FixedLengthFloatArray, Equatable {
    public static let length: Int = 15
    public var _array: [Float64]

    public init() {
        _array = [Float64](repeating: 0, count: 15)
    }

    public init(_ arr: [Float64]) {
        precondition(arr.count == 15)
        _array = arr
    }

    public init(fromBinary decoder: BinaryDecoder) throws {
        _array = try (0 ..< 15).map { _ in try Float64(from: decoder) }
    }

}



public struct FixedLengthFloat64Array36: FixedLengthFloatArray, Equatable {
    public static let length: Int = 36
    public var _array: [Float64]

    public init() {
        _array = [Float64](repeating: 0, count: 36)
    }

    public init(_ arr: [Float64]) {
        precondition(arr.count == 36)
        _array = arr
    }

    public init(fromBinary decoder: BinaryDecoder) throws {
        _array = try (0 ..< 36).map { _ in try Float64(from: decoder) }
    }

}

public struct FixedLengthFloat64Array225: FixedLengthFloatArray, Equatable {
    public static let length: Int = 225
    public var _array: [Float64]

    public init() {
        _array = [Float64](repeating: 0, count: 225)
    }

    public init(_ arr: [Float64]) {
        precondition(arr.count == 225)
        _array = arr
    }

    public init(fromBinary decoder: BinaryDecoder) throws {
        _array = try (0 ..< 225).map { _ in try Float64(from: decoder) }
    }

}


public protocol FixedLengthUInt32Array: BinaryCodable {
    static var length: Int { get }
    var _array: [UInt32] { get set }
}

public extension FixedLengthUInt32Array {
    var array: [UInt32] {
        get {
            return _array
        }
        set {
            precondition(newValue.count == Self.length)
            _array = newValue
        }
    }

    subscript(i: Int) -> UInt32 {
        get {
            return _array[i]
        }
        set {
            _array[i] = newValue
        }
    }

    // Fixed arrays has no extra length code

    func binaryEncode(to encoder: BinaryEncoder) throws {
        precondition(_array.count == Self.length)
        for a in _array {
            a.binaryEncode(to: encoder)
        }
    }
}

public struct FixedLengthUInt32Array3: FixedLengthUInt32Array, Equatable {
    public static let length: Int = 3
    public var _array: [UInt32]

    public init() {
        _array = [UInt32](repeating: 0, count: 3)
    }

    public init(_ arr: [UInt32]) {
        precondition(arr.count == 3)
        _array = arr
    }

    public init(fromBinary decoder: BinaryDecoder) throws {
        _array = try (0 ..< 3).map { _ in try UInt32(from: decoder) }
    }

}

public struct FixedLengthUInt32Array4: FixedLengthUInt32Array, Equatable {
    public static let length: Int = 4
    public var _array: [UInt32]

    public init() {
        _array = [UInt32](repeating: 0, count: 4)
    }

    public init(_ arr: [UInt32]) {
        precondition(arr.count == 4)
        _array = arr
    }

    public init(fromBinary decoder: BinaryDecoder) throws {
        _array = try (0 ..< 4).map { _ in try UInt32(from: decoder) }
    }

}

public protocol FixedLengthUInt8Array: BinaryCodable {
    static var length: Int { get }
    var _array: [UInt8] { get set }
}


public struct FixedLengthUInt8Array16: FixedLengthUInt8Array, Equatable {
    public static let length: Int = 16
    public var _array: [UInt8]

    public init() {
        _array = [UInt8](repeating: 0, count: 16)
    }

    public init(_ arr: [UInt8]) {
        precondition(arr.count == 16)
        _array = arr
    }

    public init(fromBinary decoder: BinaryDecoder) throws {
        _array = try (0 ..< 16).map { _ in try UInt8(from: decoder) }
    }

}



