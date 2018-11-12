import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension sensor_msgs {
public struct Imu: Message {
public static var md5sum: String = "6a62c6daae103f4ff57a132d6f95cec2"
public static var datatype = "sensor_msgs/Imu"
public static var definition = """
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z
"""
public static var hasHeader = false

public var header: std_msgs.header
public var orientation: geometry_msgs.Quaternion
public var orientation_covariance: [Float64]
public var angular_velocity: geometry_msgs.Vector3
public var angular_velocity_covariance: [Float64]
public var linear_acceleration: geometry_msgs.Vector3
public var linear_acceleration_covariance: [Float64]

public init(header: std_msgs.header, orientation: geometry_msgs.Quaternion, orientation_covariance: [Float64], angular_velocity: geometry_msgs.Vector3, angular_velocity_covariance: [Float64], linear_acceleration: geometry_msgs.Vector3, linear_acceleration_covariance: [Float64]) {
self.header = header
self.orientation = orientation
self.orientation_covariance = orientation_covariance
self.angular_velocity = angular_velocity
self.angular_velocity_covariance = angular_velocity_covariance
self.linear_acceleration = linear_acceleration
self.linear_acceleration_covariance = linear_acceleration_covariance
}

public init() {
    header = std_msgs.header()
orientation = geometry_msgs.Quaternion()
orientation_covariance = [Float64]()
angular_velocity = geometry_msgs.Vector3()
angular_velocity_covariance = [Float64]()
linear_acceleration = geometry_msgs.Vector3()
linear_acceleration_covariance = [Float64]()
}

}
}