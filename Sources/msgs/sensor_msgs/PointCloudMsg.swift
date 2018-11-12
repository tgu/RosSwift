import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension sensor_msgs {
public struct PointCloud: Message {
public static var md5sum: String = "d8e9c3f5afbdd8a130fd1d2763945fca"
public static var datatype = "sensor_msgs/PointCloud"
public static var definition = """
# This message holds a collection of 3d points, plus optional additional
# information about each point.

# Time of sensor data acquisition, coordinate frame ID.
Header header

# Array of 3d points. Each Point32 should be interpreted as a 3d point
# in the frame given in the header.
geometry_msgs/Point32[] points

# Each channel should have the same number of elements as points array,
# and the data in each channel should correspond 1:1 with each point.
# Channel names in common practice are listed in ChannelFloat32.msg.
ChannelFloat32[] channels
"""
public static var hasHeader = false

public var header: std_msgs.header
public var points: geometry_msgs.[Point32]
public var channels: [ChannelFloat32]

public init(header: std_msgs.header, points: geometry_msgs.[Point32], channels: [ChannelFloat32]) {
self.header = header
self.points = points
self.channels = channels
}

public init() {
    header = std_msgs.header()
points = geometry_msgs.[Point32]()
channels = [ChannelFloat32]()
}

}
}