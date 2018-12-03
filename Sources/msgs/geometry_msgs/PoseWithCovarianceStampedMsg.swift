import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
/// This expresses an estimated pose with a reference coordinate frame and timestamp
public struct PoseWithCovarianceStamped: Message {
public static var md5sum: String = "953b798c0f514ff060a53a3498ce6246"
public static var datatype = "geometry_msgs/PoseWithCovarianceStamped"
public static var definition = """
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose
"""
public static var hasHeader = false

public var header: std_msgs.header
public var pose: PoseWithCovariance

public init(header: std_msgs.header, pose: PoseWithCovariance) {
self.header = header
self.pose = pose
}

public init() {
    header = std_msgs.header()
pose = PoseWithCovariance()
}

}
}