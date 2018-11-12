import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
public struct InertiaStamped: Message {
public static var md5sum: String = "ddee48caeab5a966c5e8d166654a9ac7"
public static var datatype = "geometry_msgs/InertiaStamped"
public static var definition = """
Header header
Inertia inertia
"""
public static var hasHeader = false

public var header: std_msgs.header
public var inertia: Inertia

public init(header: std_msgs.header, inertia: Inertia) {
self.header = header
self.inertia = inertia
}

public init() {
    header = std_msgs.header()
inertia = Inertia()
}

}
}