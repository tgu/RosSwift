import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
public struct TwistWithCovarianceStamped: Message {
public static var md5sum: String = "8927a1a12fb2607ceea095b2dc440a96"
public static var datatype = "geometry_msgs/TwistWithCovarianceStamped"
public static var definition = """
# This represents an estimated twist with reference coordinate frame and timestamp.
Header header
TwistWithCovariance twist
"""
public static var hasHeader = false

public var header: std_msgs.header
public var twist: TwistWithCovariance

public init(header: std_msgs.header, twist: TwistWithCovariance) {
self.header = header
self.twist = twist
}

public init() {
    header = std_msgs.header()
twist = TwistWithCovariance()
}

}
}