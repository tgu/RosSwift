import Foundation
import StdMsgs
import RosTime


extension control_msgs {

public struct GripperCommand: Message {
public static var md5sum: String = "680acaff79486f017132a7f198d40f08"
public static var datatype = "control_msgs/GripperCommand"
public static var definition = """
float64 position
float64 max_effort
"""
public static var hasHeader = false

public var position: Float64
public var max_effort: Float64

public init(position: Float64, max_effort: Float64) {
self.position = position
self.max_effort = max_effort
}

public init() {
    position = Float64()
max_effort = Float64()
}

}
}