import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension geometry_msgs {
/// Mass [kg]
/// Center of mass [m]
/// Inertia Tensor [kg-m^2]
///     | ixx ixy ixz |
/// I = | ixy iyy iyz |
///     | ixz iyz izz |
public struct Inertia: Message {
public static var md5sum: String = "1d26e4bb6c83ff141c5cf0d883c2b0fe"
public static var datatype = "geometry_msgs/Inertia"
public static var definition = """
# Mass [kg]
float64 m

# Center of mass [m]
geometry_msgs/Vector3 com

# Inertia Tensor [kg-m^2]
#     | ixx ixy ixz |
# I = | ixy iyy iyz |
#     | ixz iyz izz |
float64 ixx
float64 ixy
float64 ixz
float64 iyy
float64 iyz
float64 izz
"""
public static var hasHeader = false

public var m: Float64
public var com: geometry_msgs.Vector3
public var ixx: Float64
public var ixy: Float64
public var ixz: Float64
public var iyy: Float64
public var iyz: Float64
public var izz: Float64

public init(m: Float64, com: geometry_msgs.Vector3, ixx: Float64, ixy: Float64, ixz: Float64, iyy: Float64, iyz: Float64, izz: Float64) {
self.m = m
self.com = com
self.ixx = ixx
self.ixy = ixy
self.ixz = ixz
self.iyy = iyy
self.iyz = iyz
self.izz = izz
}

public init() {
    m = Float64()
com = geometry_msgs.Vector3()
ixx = Float64()
ixy = Float64()
ixz = Float64()
iyy = Float64()
iyz = Float64()
izz = Float64()
}

}
}