import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
public struct PoseWithCovariance: Message {
public static var md5sum: String = "c23e848cf1b7533a8d7c259073a97e6f"
public static var datatype = "geometry_msgs/PoseWithCovariance"
public static var definition = """
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
"""
public static var hasHeader = false

public var pose: Pose
public var covariance: [Float64]

public init(pose: Pose, covariance: [Float64]) {
self.pose = pose
self.covariance = covariance
}

public init() {
    pose = Pose()
covariance = [Float64]()
}

}
}