import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension nav_msgs {
public struct Path: Message {
public static var md5sum: String = "6227e2b7e9cce15051f669a5e197bbf7"
public static var datatype = "nav_msgs/Path"
public static var definition = """
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses
"""
public static var hasHeader = false

public var header: std_msgs.header
public var poses: geometry_msgs.[PoseStamped]

public init(header: std_msgs.header, poses: geometry_msgs.[PoseStamped]) {
self.header = header
self.poses = poses
}

public init() {
    header = std_msgs.header()
poses = geometry_msgs.[PoseStamped]()
}

}
}