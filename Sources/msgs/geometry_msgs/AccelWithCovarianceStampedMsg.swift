import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
/// This represents an estimated accel with reference coordinate frame and timestamp.
public struct AccelWithCovarianceStamped: Message {
public static var md5sum: String = "96adb295225031ec8d57fb4251b0a886"
public static var datatype = "geometry_msgs/AccelWithCovarianceStamped"
public static var definition = """
# This represents an estimated accel with reference coordinate frame and timestamp.
Header header
AccelWithCovariance accel
"""
public static var hasHeader = false

public var header: std_msgs.header
public var accel: AccelWithCovariance

public init(header: std_msgs.header, accel: AccelWithCovariance) {
self.header = header
self.accel = accel
}

public init() {
    header = std_msgs.header()
accel = AccelWithCovariance()
}

}
}