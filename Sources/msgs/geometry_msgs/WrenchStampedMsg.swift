import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
/// A wrench with reference coordinate frame and timestamp
public struct WrenchStamped: Message {
public static var md5sum: String = "d78d3cb249ce23087ade7e7d0c40cfa7"
public static var datatype = "geometry_msgs/WrenchStamped"
public static var definition = """
# A wrench with reference coordinate frame and timestamp
Header header
Wrench wrench
"""
public static var hasHeader = false

public var header: std_msgs.header
public var wrench: Wrench

public init(header: std_msgs.header, wrench: Wrench) {
self.header = header
self.wrench = wrench
}

public init() {
    header = std_msgs.header()
wrench = Wrench()
}

}
}