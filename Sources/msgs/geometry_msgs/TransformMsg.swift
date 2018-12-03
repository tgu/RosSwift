import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
/// This represents the transform between two coordinate frames in free space.
public struct Transform: Message {
public static var md5sum: String = "ac9eff44abf714214112b05d54a3cf9b"
public static var datatype = "geometry_msgs/Transform"
public static var definition = """
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
"""
public static var hasHeader = false

public var translation: Vector3
public var rotation: Quaternion

public init(translation: Vector3, rotation: Quaternion) {
self.translation = translation
self.rotation = rotation
}

public init() {
    translation = Vector3()
rotation = Quaternion()
}

}
}