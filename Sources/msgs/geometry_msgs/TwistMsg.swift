import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
/// This expresses velocity in free space broken into its linear and angular parts.
public struct Twist: Message {
public static var md5sum: String = "9f195f881246fdfa2798d1d3eebca84a"
public static var datatype = "geometry_msgs/Twist"
public static var definition = """
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
"""
public static var hasHeader = false

public var linear: Vector3
public var angular: Vector3

public init(linear: Vector3, angular: Vector3) {
self.linear = linear
self.angular = angular
}

public init() {
    linear = Vector3()
angular = Vector3()
}

}
}