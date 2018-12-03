import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
/// This represents a Polygon with reference coordinate frame and timestamp
public struct PolygonStamped: Message {
public static var md5sum: String = "c6be8f7dc3bee7fe9e8d296070f53340"
public static var datatype = "geometry_msgs/PolygonStamped"
public static var definition = """
# This represents a Polygon with reference coordinate frame and timestamp
Header header
Polygon polygon
"""
public static var hasHeader = false

public var header: std_msgs.header
public var polygon: Polygon

public init(header: std_msgs.header, polygon: Polygon) {
self.header = header
self.polygon = polygon
}

public init() {
    header = std_msgs.header()
polygon = Polygon()
}

}
}