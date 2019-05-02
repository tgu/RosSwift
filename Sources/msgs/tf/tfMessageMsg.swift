import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension tf {

public struct tfMessage: Message {
public static var md5sum: String = ""
public static var datatype = "tf/tfMessage"
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
