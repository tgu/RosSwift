import Foundation
import StdMsgs
import RosTime


extension actionlib_msgs {
///Allow for the user to associate a string with GoalStatus for debugging
public struct GoalStatus: Message {
public static var md5sum: String = "d388f9b87b3c471f784434d671988d4a"
public static var datatype = "actionlib_msgs/GoalStatus"
public static var definition = """
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string text
"""
public static var hasHeader = false

public var goal_id: GoalID
public var status: UInt8
public let PENDING: UInt8 = 0
public let ACTIVE: UInt8 = 1
public let PREEMPTED: UInt8 = 2
public let SUCCEEDED: UInt8 = 3
public let ABORTED: UInt8 = 4
public let REJECTED: UInt8 = 5
public let PREEMPTING: UInt8 = 6
public let RECALLING: UInt8 = 7
public let RECALLED: UInt8 = 8
public let LOST: UInt8 = 9
public var text: String

public init(goal_id: GoalID, status: UInt8, text: String) {
self.goal_id = goal_id
self.status = status
self.text = text
}

public init() {
    goal_id = GoalID()
status = UInt8()
text = String()
}

}
}