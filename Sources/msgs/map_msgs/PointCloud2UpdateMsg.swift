import Foundation
import StdMsgs
import RosTime
import sensor_msgs

extension map_msgs {

public struct PointCloud2Update: Message {
public static var md5sum: String = "6c58e4f249ae9cd2b24fb1ee0f99195e"
public static var datatype = "map_msgs/PointCloud2Update"
public static var definition = """
uint32 ADD=0
uint32 DELETE=1
Header header
uint32 type          # type of update, one of ADD or DELETE
sensor_msgs/PointCloud2 points
"""
public static var hasHeader = false

public var ADD=0: UInt32
public var DELETE=1: UInt32
public var header: std_msgs.header
public var type: UInt32
public var points: sensor_msgs.PointCloud2

public init(ADD=0: UInt32, DELETE=1: UInt32, header: std_msgs.header, type: UInt32, points: sensor_msgs.PointCloud2) {
self.ADD=0 = ADD=0
self.DELETE=1 = DELETE=1
self.header = header
self.type = type
self.points = points
}

public init() {
    ADD=0 = UInt32()
DELETE=1 = UInt32()
header = std_msgs.header()
type = UInt32()
points = sensor_msgs.PointCloud2()
}

}
}
