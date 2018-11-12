import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
public struct TransformStamped: Message {
public static var md5sum: String = "b5764a33bfeb3588febc2682852579b0"
public static var datatype = "geometry_msgs/TransformStamped"
public static var definition = """
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href="http://wiki.ros.org/tf">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform
"""
public static var hasHeader = false

public var header: std_msgs.header
public var child_frame_id: String
public var transform: Transform

public init(header: std_msgs.header, child_frame_id: String, transform: Transform) {
self.header = header
self.child_frame_id = child_frame_id
self.transform = transform
}

public init() {
    header = std_msgs.header()
child_frame_id = String()
transform = Transform()
}

}
}