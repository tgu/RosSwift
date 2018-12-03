import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
/// An accel with reference coordinate frame and timestamp
public struct AccelStamped: Message {
public static var md5sum: String = "d8a98a5d81351b6eb0578c78557e7659"
public static var datatype = "geometry_msgs/AccelStamped"
public static var definition = """
# An accel with reference coordinate frame and timestamp
Header header
Accel accel
"""
public static var hasHeader = false

public var header: std_msgs.header
public var accel: Accel

public init(header: std_msgs.header, accel: Accel) {
self.header = header
self.accel = accel
}

public init() {
    header = std_msgs.header()
accel = Accel()
}

}
}