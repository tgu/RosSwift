import Foundation
import StdMsgs
import RosTime


extension sensor_msgs {
public struct JoyFeedbackArray: Message {
public static var md5sum: String = "cde5730a895b1fc4dee6f91b754b213d"
public static var datatype = "sensor_msgs/JoyFeedbackArray"
public static var definition = """
# This message publishes values for multiple feedback at once. 
JoyFeedback[] array
"""
public static var hasHeader = false

public var array: [JoyFeedback]

public init(array: [JoyFeedback]) {
self.array = array
}

public init() {
    array = [JoyFeedback]()
}

}
}