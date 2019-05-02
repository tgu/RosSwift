import Foundation
import StdMsgs
import RosTime


extension actionlib_msgs {
	///Allow for the user to associate a string with GoalStatus for debugging
	public struct GoalStatus: Message {
		public static let md5sum: String = "4e2255c18eb3b8637ff3a4f4733a7199"
		public static let datatype = "actionlib_msgs/GoalStatus"
		public static let definition = """
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

		public static let hasHeader = false

		public static let PENDING : UInt8 = 0
		public static let ACTIVE : UInt8 = 1
		public static let PREEMPTED : UInt8 = 2
		public static let SUCCEEDED : UInt8 = 3
		public static let ABORTED : UInt8 = 4
		public static let REJECTED : UInt8 = 5
		public static let PREEMPTING : UInt8 = 6
		public static let RECALLING : UInt8 = 7
		public static let RECALLED : UInt8 = 8
		public static let LOST : UInt8 = 9
		public var goal_id: GoalID
		public var status: UInt8
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