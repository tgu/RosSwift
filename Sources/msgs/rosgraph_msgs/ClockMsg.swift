import Foundation
import StdMsgs
import RosTime


extension rosgraph_msgs {
	/// roslib/Clock is used for publishing simulated time in ROS. 
	/// This message simply communicates the current time.
	/// For more information, see http://www.ros.org/wiki/Clock
	public struct Clock: Message {
		public static var md5sum: String = "a9c97c1d230cfc112e270351a944ee47"
		public static var datatype = "rosgraph_msgs/Clock"
		public static var definition = """
			# roslib/Clock is used for publishing simulated time in ROS. 
			# This message simply communicates the current time.
			# For more information, see http://www.ros.org/wiki/Clock
			time clock
			"""

		public static let hasHeader = false

		public var clock: Time

		public init(clock: Time) {
			self.clock = clock
		}

		public init() {
			clock = Time()
		}
	}
}