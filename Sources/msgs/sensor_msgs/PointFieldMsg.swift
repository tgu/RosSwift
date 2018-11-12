import Foundation
import StdMsgs
import RosTime


extension sensor_msgs {
public struct PointField: Message {
public static var md5sum: String = "268eacb2962780ceac86cbd17e328150"
public static var datatype = "sensor_msgs/PointField"
public static var definition = """
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
"""
public static var hasHeader = false

public let INT8 : UInt8 = 1
public let UINT8 : UInt8 = 2
public let INT16 : UInt8 = 3
public let UINT16 : UInt8 = 4
public let INT32 : UInt8 = 5
public let UINT32 : UInt8 = 6
public let FLOAT32 : UInt8 = 7
public let FLOAT64 : UInt8 = 8
public var name: String
public var offset: UInt32
public var datatype: UInt8
public var count: UInt32

public init(name: String, offset: UInt32, datatype: UInt8, count: UInt32) {
self.name = name
self.offset = offset
self.datatype = datatype
self.count = count
}

public init() {
    name = String()
offset = UInt32()
datatype = UInt8()
count = UInt32()
}

}
}