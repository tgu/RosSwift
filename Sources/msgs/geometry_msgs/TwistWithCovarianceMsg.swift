import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
/// This expresses velocity in free space with uncertainty.
/// Row-major representation of the 6x6 covariance matrix
/// The orientation parameters use a fixed-axis representation.
/// In order, the parameters are:
/// (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
public struct TwistWithCovariance: Message {
public static var md5sum: String = "1fe8a28e6890a4cc3ae4c3ca5c7d82e6"
public static var datatype = "geometry_msgs/TwistWithCovariance"
public static var definition = """
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
"""
public static var hasHeader = false

public var twist: Twist
public var covariance: [Float64]

public init(twist: Twist, covariance: [Float64]) {
self.twist = twist
self.covariance = covariance
}

public init() {
    twist = Twist()
covariance = [Float64]()
}

}
}