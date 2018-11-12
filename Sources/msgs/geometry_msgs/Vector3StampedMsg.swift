import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
public struct Vector3Stamped: Message {
public static var md5sum: String = "7b324c7325e683bf02a9b14b01090ec7"
public static var datatype = "geometry_msgs/Vector3Stamped"
public static var definition = """
# This represents a Vector3 with reference coordinate frame and timestamp
Header header
Vector3 vector
"""
public static var hasHeader = false

public var header: std_msgs.header
public var vector: Vector3

public init(header: std_msgs.header, vector: Vector3) {
self.header = header
self.vector = vector
}

public init() {
    header = std_msgs.header()
vector = Vector3()
}

}
}