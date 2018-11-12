import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
public struct Wrench: Message {
public static var md5sum: String = "4f539cf138b23283b520fd271b567936"
public static var datatype = "geometry_msgs/Wrench"
public static var definition = """
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque
"""
public static var hasHeader = false

public var force: Vector3
public var torque: Vector3

public init(force: Vector3, torque: Vector3) {
self.force = force
self.torque = torque
}

public init() {
    force = Vector3()
torque = Vector3()
}

}
}