import Foundation
import StdMsgs
import RosTime


extension pcl_msgs {

public struct PointIndices: Message {
public static var md5sum: String = "458c7998b7eaf99908256472e273b3d4"
public static var datatype = "pcl_msgs/PointIndices"
public static var definition = """
Header header
int32[] indices
"""
public static var hasHeader = false

public var header: std_msgs.header
public var indices: [Int32]

public init(header: std_msgs.header, indices: [Int32]) {
self.header = header
self.indices = indices
}

public init() {
    header = std_msgs.header()
indices = [Int32]()
}

}
}
