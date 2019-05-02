import Foundation
import StdMsgs
import RosTime


extension pcl_msgs {
/// List of point indices
public struct Vertices: Message {
public static var md5sum: String = "39bd7b1c23763ddd1b882b97cb7cfe11"
public static var datatype = "pcl_msgs/Vertices"
public static var definition = """
# List of point indices
uint32[] vertices
"""
public static var hasHeader = false

public var vertices: [UInt32]

public init(vertices: [UInt32]) {
self.vertices = vertices
}

public init() {
    vertices = [UInt32]()
}

}
}
