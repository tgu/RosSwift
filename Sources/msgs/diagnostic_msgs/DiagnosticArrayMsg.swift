import Foundation
import StdMsgs
import RosTime


extension diagnostic_msgs {
/// This message is used to send diagnostic information about the state of the robot
public struct DiagnosticArray: Message {
public static var md5sum: String = "60810da900de1dd6ddd437c3503511da"
public static var datatype = "diagnostic_msgs/DiagnosticArray"
public static var definition = """
# This message is used to send diagnostic information about the state of the robot
Header header #for timestamp
DiagnosticStatus[] status # an array of components being reported on
"""
public static var hasHeader = false

public var header: std_msgs.header
public var status: [DiagnosticStatus]

public init(header: std_msgs.header, status: [DiagnosticStatus]) {
self.header = header
self.status = status
}

public init() {
    header = std_msgs.header()
status = [DiagnosticStatus]()
}

}
}