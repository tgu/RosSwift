import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension nav_msgs {
	/// This represents an estimate of a position and velocity in free space.  
	/// The pose in this message should be specified in the coordinate frame given by header.frame_id.
	/// The twist in this message should be specified in the coordinate frame given by the child_frame_id
	public struct Odometry: Message {
		public static let md5sum: String = "cd5e73d190d741a2f92e81eda573aca7"
		public static let datatype = "nav_msgs/Odometry"
		public static let definition = """
			# This represents an estimate of a position and velocity in free space.  
			# The pose in this message should be specified in the coordinate frame given by header.frame_id.
			# The twist in this message should be specified in the coordinate frame given by the child_frame_id
			Header header
			string child_frame_id
			geometry_msgs/PoseWithCovariance pose
			geometry_msgs/TwistWithCovariance twist
			"""

		public static let hasHeader = true

	
		public var header: std_msgs.Header
		public var child_frame_id: String
		public var pose: geometry_msgs.PoseWithCovariance
		public var twist: geometry_msgs.TwistWithCovariance

		public init(header: std_msgs.Header, child_frame_id: String, pose: geometry_msgs.PoseWithCovariance, twist: geometry_msgs.TwistWithCovariance) {
			self.header = header
			self.child_frame_id = child_frame_id
			self.pose = pose
			self.twist = twist
		}

		public init() {
			header = std_msgs.Header()
			child_frame_id = String()
			pose = geometry_msgs.PoseWithCovariance()
			twist = geometry_msgs.TwistWithCovariance()
		}
	}
}