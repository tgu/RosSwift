import Foundation
import StdMsgs
import RosTime
import trajectory_msgs

extension control_msgs {

	public struct JointTrajectoryControllerState: Message {
		public static let md5sum: String = "10817c60c2486ef6b33e97dcd87f4474"
		public static let datatype = "control_msgs/JointTrajectoryControllerState"
		public static let definition = """
			Header header
			string[] joint_names
			trajectory_msgs/JointTrajectoryPoint desired
			trajectory_msgs/JointTrajectoryPoint actual
			trajectory_msgs/JointTrajectoryPoint error  # Redundant, but useful
			"""

		public static let hasHeader = false

	
		public var header: std_msgs.Header
		public var joint_names: [String]
		public var desired: trajectory_msgs.JointTrajectoryPoint
		public var actual: trajectory_msgs.JointTrajectoryPoint
		public var error: trajectory_msgs.JointTrajectoryPoint

		public init(header: std_msgs.Header, joint_names: [String], desired: trajectory_msgs.JointTrajectoryPoint, actual: trajectory_msgs.JointTrajectoryPoint, error: trajectory_msgs.JointTrajectoryPoint) {
			self.header = header
			self.joint_names = joint_names
			self.desired = desired
			self.actual = actual
			self.error = error
		}

		public init() {
			header = std_msgs.Header()
			joint_names = [String]()
			desired = trajectory_msgs.JointTrajectoryPoint()
			actual = trajectory_msgs.JointTrajectoryPoint()
			error = trajectory_msgs.JointTrajectoryPoint()
		}
	}
}