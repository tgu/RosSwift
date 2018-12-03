import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
/// An array of poses with a header for global reference.
public struct PoseArray: Message {
public static var md5sum: String = "916c28c5764443f268b296bb671b9d97"
public static var datatype = "geometry_msgs/PoseArray"
public static var definition = """
# An array of poses with a header for global reference.

Header header

Pose[] poses
"""
public static var hasHeader = false

public var header: std_msgs.header
public var poses: [Pose]

public init(header: std_msgs.header, poses: [Pose]) {
self.header = header
self.poses = poses
}

public init() {
    header = std_msgs.header()
poses = [Pose]()
}

}
}