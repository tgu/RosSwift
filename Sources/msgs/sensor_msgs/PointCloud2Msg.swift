import Foundation
import StdMsgs
import RosTime


extension sensor_msgs {
public struct PointCloud2: Message {
public static var md5sum: String = "1158d486dd51d683ce2f1be655c3c181"
public static var datatype = "sensor_msgs/PointCloud2"
public static var definition = """
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points
"""
public static var hasHeader = false

public var header: std_msgs.header
public var height: UInt32
public var width: UInt32
public var fields: [PointField]
public var is_bigendian: Bool
public var point_step: UInt32
public var row_step: UInt32
public var data: [UInt8]
public var is_dense: Bool

public init(header: std_msgs.header, height: UInt32, width: UInt32, fields: [PointField], is_bigendian: Bool, point_step: UInt32, row_step: UInt32, data: [UInt8], is_dense: Bool) {
self.header = header
self.height = height
self.width = width
self.fields = fields
self.is_bigendian = is_bigendian
self.point_step = point_step
self.row_step = row_step
self.data = data
self.is_dense = is_dense
}

public init() {
    header = std_msgs.header()
height = UInt32()
width = UInt32()
fields = [PointField]()
is_bigendian = Bool()
point_step = UInt32()
row_step = UInt32()
data = [UInt8]()
is_dense = Bool()
}

}
}