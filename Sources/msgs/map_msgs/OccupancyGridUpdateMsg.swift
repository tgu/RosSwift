import Foundation
import StdMsgs
import RosTime


extension map_msgs {

public struct OccupancyGridUpdate: Message {
public static var md5sum: String = "b295be292b335c34718bd939deebe1c9"
public static var datatype = "map_msgs/OccupancyGridUpdate"
public static var definition = """
Header header
int32 x
int32 y
uint32 width
uint32 height
int8[] data
"""
public static var hasHeader = false

public var header: std_msgs.header
public var x: Int32
public var y: Int32
public var width: UInt32
public var height: UInt32
public var data: [Int8]

public init(header: std_msgs.header, x: Int32, y: Int32, width: UInt32, height: UInt32, data: [Int8]) {
self.header = header
self.x = x
self.y = y
self.width = width
self.height = height
self.data = data
}

public init() {
    header = std_msgs.header()
x = Int32()
y = Int32()
width = UInt32()
height = UInt32()
data = [Int8]()
}

}
}
