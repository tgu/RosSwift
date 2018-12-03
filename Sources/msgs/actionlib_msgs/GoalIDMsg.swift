import Foundation
import StdMsgs
import RosTime


extension actionlib_msgs {
/// The stamp should store the time at which this goal was requested.
/// It is used by an action server when it tries to preempt all
/// goals that were requested before a certain time
/// The id provides a way to associate feedback and
/// result message with specific goal requests. The id
/// specified must be unique.
public struct GoalID: Message {
public static var md5sum: String = "302881f31927c1df708a2dbab0e80ee8"
public static var datatype = "actionlib_msgs/GoalID"
public static var definition = """
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id
"""
public static var hasHeader = false

public var stamp: RosTime.TimeBase
public var id: String

public init(stamp: RosTime.TimeBase, id: String) {
self.stamp = stamp
self.id = id
}

public init() {
    stamp = RosTime.TimeBase()
id = String()
}

}
}