import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
///A specification of a polygon where the first and last points are assumed to be connected
public struct Polygon: Message {
public static var md5sum: String = "cd60a26494a087f577976f0329fa120e"
public static var datatype = "geometry_msgs/Polygon"
public static var definition = """
#A specification of a polygon where the first and last points are assumed to be connected
Point32[] points
"""
public static var hasHeader = false

public var points: [Point32]

public init(points: [Point32]) {
self.points = points
}

public init() {
    points = [Point32]()
}

}
}