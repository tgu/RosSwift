import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
public struct Pose2D: Message {
public static var md5sum: String = "938fa65709584ad8e77d238529be13b8"
public static var datatype = "geometry_msgs/Pose2D"
public static var definition = """
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
"""
public static var hasHeader = false

public var x: Float64
public var y: Float64
public var theta: Float64

public init(x: Float64, y: Float64, theta: Float64) {
self.x = x
self.y = y
self.theta = theta
}

public init() {
    x = Float64()
y = Float64()
theta = Float64()
}

}
}