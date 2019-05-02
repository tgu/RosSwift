import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension tf2_msgs {

public struct TFMessage: Message {
public static var md5sum: String = "94810edda583a504dfda3829e70d7eec"
public static var datatype = "tf2_msgs/TFMessage"
public static var definition = """
geometry_msgs/TransformStamped[] transforms
"""
public static var hasHeader = false

public var transforms: geometry_msgs.[TransformStamped]

public init(transforms: geometry_msgs.[TransformStamped]) {
self.transforms = transforms
}

public init() {
    transforms = geometry_msgs.[TransformStamped]()
}

}
}
