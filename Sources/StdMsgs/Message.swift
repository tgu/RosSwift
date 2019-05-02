//
//  Message.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation
import RosTime
import BinaryCoder

public protocol Message: BinaryCodable {
    static var md5sum : String { get }
    static var datatype : String { get }
    static var hasHeader : Bool { get }
    static var definition: String { get }
}


// Builtin native types

extension Bool: Message {
    public static let md5sum = std_msgs.bool.md5sum
    public static let datatype = std_msgs.bool.datatype
    public static let hasHeader = false
    public static let definition = std_msgs.bool.definition
}

extension Int8: Message {
    public static let md5sum = std_msgs.int8.md5sum
    public static let datatype = std_msgs.int8.datatype
    public static let hasHeader = false
    public static let definition = std_msgs.int8.definition
}

extension Int16: Message {
    public static let md5sum = std_msgs.int16.md5sum
    public static let datatype = std_msgs.int16.datatype
    public static let hasHeader = false
    public static let definition = std_msgs.int16.definition
}

extension Int32: Message {
    public static let md5sum = std_msgs.int32.md5sum
    public static let datatype = std_msgs.int32.datatype
    public static let hasHeader = false
    public static let definition = std_msgs.int32.definition
}

extension Int64: Message {
    public static let md5sum = std_msgs.int64.md5sum
    public static let datatype = std_msgs.int64.datatype
    public static let hasHeader = false
    public static let definition = std_msgs.int64.definition
}

extension UInt8: Message {
    public static let md5sum = std_msgs.uint8.md5sum
    public static let datatype = std_msgs.uint8.datatype
    public static let hasHeader = false
    public static let definition = std_msgs.uint8.definition
}

extension UInt16: Message {
    public static let md5sum = std_msgs.uint16.md5sum
    public static let datatype = std_msgs.uint16.datatype
    public static let hasHeader = false
    public static let definition = std_msgs.uint16.definition
}

extension UInt32: Message {
    public static let md5sum = std_msgs.uint32.md5sum
    public static let datatype = std_msgs.uint32.datatype
    public static let hasHeader = false
    public static let definition = std_msgs.uint32.definition
}

extension UInt64: Message {
    public static let md5sum = std_msgs.uint64.md5sum
    public static let datatype = std_msgs.uint64.datatype
    public static let hasHeader = false
    public static let definition = std_msgs.uint64.definition
}

extension Float32: Message {
    public static let md5sum = std_msgs.float32.md5sum
    public static let datatype = std_msgs.float32.datatype
    public static let hasHeader = false
    public static let definition = std_msgs.float32.definition
}

extension Float64: Message {
    public static let md5sum = std_msgs.float64.md5sum
    public static let datatype = std_msgs.float64.datatype
    public static let hasHeader = false
    public static let definition = std_msgs.float64.definition
}

extension String: Message {
    public static let md5sum = std_msgs.string.md5sum
    public static let datatype = std_msgs.string.datatype
    public static let hasHeader = false
    public static let definition = std_msgs.string.definition
}
