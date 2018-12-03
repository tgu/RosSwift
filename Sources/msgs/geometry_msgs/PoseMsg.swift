import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
/// A representation of pose in free space, composed of position and orientation. 
public struct Pose: Message {
public static var md5sum: String = "e45d45a5a1ce597b249e23fb30fc871f"
public static var datatype = "geometry_msgs/Pose"
public static var definition = """
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation
"""
public static var hasHeader = false

public var position: Point
public var orientation: Quaternion

public init(position: Point, orientation: Quaternion) {
self.position = position
self.orientation = orientation
}

public init() {
    position = Point()
orientation = Quaternion()
}

}
}