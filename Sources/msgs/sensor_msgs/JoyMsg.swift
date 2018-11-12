import Foundation
import StdMsgs
import RosTime


extension sensor_msgs {
public struct Joy: Message {
public static var md5sum: String = "5a9ea5f83505693b71e785041e67a8bb"
public static var datatype = "sensor_msgs/Joy"
public static var definition = """
# Reports the state of a joysticks axes and buttons.
Header header           # timestamp in the header is the time the data is received from the joystick
float32[] axes          # the axes measurements from a joystick
int32[] buttons         # the buttons measurements from a joystick
"""
public static var hasHeader = false

public var header: std_msgs.header
public var axes: [Float32]
public var buttons: [Int32]

public init(header: std_msgs.header, axes: [Float32], buttons: [Int32]) {
self.header = header
self.axes = axes
self.buttons = buttons
}

public init() {
    header = std_msgs.header()
axes = [Float32]()
buttons = [Int32]()
}

}
}