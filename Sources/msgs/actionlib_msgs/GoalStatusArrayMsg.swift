import Foundation
import StdMsgs
import RosTime


extension actionlib_msgs {
/// Stores the statuses for goals that are currently being tracked
/// by an action server
public struct GoalStatusArray: Message {
public static var md5sum: String = "8b2b82f13216d0a8ea88bd3af735e619"
public static var datatype = "actionlib_msgs/GoalStatusArray"
public static var definition = """
# Stores the statuses for goals that are currently being tracked
# by an action server
Header header
GoalStatus[] status_list
"""
public static var hasHeader = false

public var header: std_msgs.header
public var status_list: [GoalStatus]

public init(header: std_msgs.header, status_list: [GoalStatus]) {
self.header = header
self.status_list = status_list
}

public init() {
    header = std_msgs.header()
status_list = [GoalStatus]()
}

}
}