import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
/// This expresses acceleration in free space with uncertainty.
/// Row-major representation of the 6x6 covariance matrix
/// The orientation parameters use a fixed-axis representation.
/// In order, the parameters are:
/// (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
public struct AccelWithCovariance: Message {
public static var md5sum: String = "ad5a718d699c6be72a02b8d6a139f334"
public static var datatype = "geometry_msgs/AccelWithCovariance"
public static var definition = """
# This expresses acceleration in free space with uncertainty.

Accel accel

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
"""
public static var hasHeader = false

public var accel: Accel
public var covariance: [Float64]

public init(accel: Accel, covariance: [Float64]) {
self.accel = accel
self.covariance = covariance
}

public init() {
    accel = Accel()
covariance = [Float64]()
}

}
}