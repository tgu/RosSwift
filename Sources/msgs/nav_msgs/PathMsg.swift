import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension nav_msgs {
	///An array of poses that represents a Path for a robot to follow
	public struct Path: Message {
		public static let md5sum: String = "6227e2b7e9cce15051f669a5e197bbf7"
		public static let datatype = "nav_msgs/Path"
		public static let definition = """
			#An array of poses that represents a Path for a robot to follow
			Header header
			geometry_msgs/PoseStamped[] poses
			"""

		public static let hasHeader = true

	
		public var header: std_msgs.Header
		public var poses: [geometry_msgs.PoseStamped]

		public init(header: std_msgs.Header, poses: [geometry_msgs.PoseStamped]) {
			self.header = header
			self.poses = poses
		}

		public init() {
			header = std_msgs.Header()
			poses = [geometry_msgs.PoseStamped]()
		}
	}
}