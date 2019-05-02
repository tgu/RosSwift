import Foundation
import StdMsgs
import RosTime
import sensor_msgs

extension pcl_msgs {
/// Separate header for the polygonal surface
/// Vertices of the mesh as a point cloud
/// List of polygons
public struct PolygonMesh: Message {
public static var md5sum: String = "45a5fc6ad2cde8489600a790acc9a38a"
public static var datatype = "pcl_msgs/PolygonMesh"
public static var definition = """
# Separate header for the polygonal surface
Header header
# Vertices of the mesh as a point cloud
sensor_msgs/PointCloud2 cloud
# List of polygons
Vertices[] polygons
"""
public static var hasHeader = false

public var header: std_msgs.header
public var cloud: sensor_msgs.PointCloud2
public var polygons: [Vertices]

public init(header: std_msgs.header, cloud: sensor_msgs.PointCloud2, polygons: [Vertices]) {
self.header = header
self.cloud = cloud
self.polygons = polygons
}

public init() {
    header = std_msgs.header()
cloud = sensor_msgs.PointCloud2()
polygons = [Vertices]()
}

}
}
