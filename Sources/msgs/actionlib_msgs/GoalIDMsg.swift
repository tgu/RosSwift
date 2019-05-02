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
		public static let md5sum: String = "302881f31927c1df708a2dbab0e80ee8"
		public static let datatype = "actionlib_msgs/GoalID"
		public static let definition = """
			# The stamp should store the time at which this goal was requested.
			# It is used by an action server when it tries to preempt all
			# goals that were requested before a certain time
			time stamp
			# The id provides a way to associate feedback and
			# result message with specific goal requests. The id
			# specified must be unique.
			string id
			"""

		public static let hasHeader = false

	
		public var stamp: Time
		public var id: String

		public init(stamp: Time, id: String) {
			self.stamp = stamp
			self.id = id
		}

		public init() {
			stamp = Time()
			id = String()
		}
	}
}