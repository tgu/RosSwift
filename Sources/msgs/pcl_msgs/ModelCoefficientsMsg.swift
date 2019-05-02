import Foundation
import StdMsgs
import RosTime


extension pcl_msgs {

public struct ModelCoefficients: Message {
public static var md5sum: String = "ca27dea75e72cb894cd36f9e5005e93e"
public static var datatype = "pcl_msgs/ModelCoefficients"
public static var definition = """
Header header
float32[] values
"""
public static var hasHeader = false

public var header: std_msgs.header
public var values: [Float32]

public init(header: std_msgs.header, values: [Float32]) {
self.header = header
self.values = values
}

public init() {
    header = std_msgs.header()
values = [Float32]()
}

}
}
