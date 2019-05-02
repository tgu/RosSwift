import Foundation
import StdMsgs
import RosTime


extension trajectory_msgs {

	public struct JointTrajectory: Message {
		public static let md5sum: String = "65b4f94a94d1ed67169da35a02f33d3f"
		public static let datatype = "trajectory_msgs/JointTrajectory"
		public static let definition = """
			Header header
			string[] joint_names
			JointTrajectoryPoint[] points
			"""

		public static let hasHeader = true

	
		public var header: std_msgs.Header
		public var joint_names: [String]
		public var points: [JointTrajectoryPoint]

		public init(header: std_msgs.Header, joint_names: [String], points: [JointTrajectoryPoint]) {
			self.header = header
			self.joint_names = joint_names
			self.points = points
		}

		public init() {
			header = std_msgs.Header()
			joint_names = [String]()
			points = [JointTrajectoryPoint]()
		}
	}
}