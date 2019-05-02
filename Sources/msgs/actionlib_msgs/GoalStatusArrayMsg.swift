import Foundation
import StdMsgs
import RosTime


extension actionlib_msgs {
	/// Stores the statuses for goals that are currently being tracked
	/// by an action server
	public struct GoalStatusArray: Message {
		public static let md5sum: String = "804691eecc31f3b5132cfbc7399e4177"
		public static let datatype = "actionlib_msgs/GoalStatusArray"
		public static let definition = """
			# Stores the statuses for goals that are currently being tracked
			# by an action server
			Header header
			GoalStatus[] status_list
			"""

		public static let hasHeader = true

	
		public var header: std_msgs.Header
		public var status_list: [GoalStatus]

		public init(header: std_msgs.Header, status_list: [GoalStatus]) {
			self.header = header
			self.status_list = status_list
		}

		public init() {
			header = std_msgs.Header()
			status_list = [GoalStatus]()
		}
	}
}