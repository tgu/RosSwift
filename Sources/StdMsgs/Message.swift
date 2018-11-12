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


struct MessageHeader: Message {
    static var md5sum = "2176decaecbce78abc3b96ef049fabed"
    static var datatype = "std_msgs/Header"
    static var hasHeader = false
    static var definition = """
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data
    # in a particular coordinate frame.
    #
    # sequence ID: consecutively increasing ID
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    """


    let seq : UInt32
    let stamp : RosTime.Time
    let frame_id : UInt8
}

public struct Empty: BinaryCodable {}



// Builtin types

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
