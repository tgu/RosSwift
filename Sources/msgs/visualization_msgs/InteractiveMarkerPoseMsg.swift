import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension visualization_msgs {
	/// Time/frame info.
	/// Initial pose. Also, defines the pivot point for rotations.
	/// Identifying string. Must be globally unique in
	/// the topic that this message is sent through.
	public struct InteractiveMarkerPose: Message {
		public static let md5sum: String = "a6e6833209a196a38d798dadb02c81f8"
		public static let datatype = "visualization_msgs/InteractiveMarkerPose"
		public static let definition = """
			# Time/frame info.
			Header header
			# Initial pose. Also, defines the pivot point for rotations.
			geometry_msgs/Pose pose
			# Identifying string. Must be globally unique in
			# the topic that this message is sent through.
			string name
			"""

		public static let hasHeader = true

	
		public var header: std_msgs.Header
		public var pose: geometry_msgs.Pose
		public var name: String

		public init(header: std_msgs.Header, pose: geometry_msgs.Pose, name: String) {
			self.header = header
			self.pose = pose
			self.name = name
		}

		public init() {
			header = std_msgs.Header()
			pose = geometry_msgs.Pose()
			name = String()
		}
	}
}