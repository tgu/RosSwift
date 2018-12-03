import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
/// A twist with reference coordinate frame and timestamp
public struct TwistStamped: Message {
public static var md5sum: String = "98d34b0043a2093cf9d9345ab6eef12e"
public static var datatype = "geometry_msgs/TwistStamped"
public static var definition = """
# A twist with reference coordinate frame and timestamp
Header header
Twist twist
"""
public static var hasHeader = false

public var header: std_msgs.header
public var twist: Twist

public init(header: std_msgs.header, twist: Twist) {
self.header = header
self.twist = twist
}

public init() {
    header = std_msgs.header()
twist = Twist()
}

}
}