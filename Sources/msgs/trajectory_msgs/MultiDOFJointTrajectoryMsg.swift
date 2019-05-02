import Foundation
import StdMsgs
import RosTime


extension trajectory_msgs {
	/// The header is used to specify the coordinate frame and the reference time for the trajectory durations
	/// A representation of a multi-dof joint trajectory (each point is a transformation)
	/// Each point along the trajectory will include an array of positions/velocities/accelerations
	/// that has the same length as the array of joint names, and has the same order of joints as 
	/// the joint names array.
	public struct MultiDOFJointTrajectory: Message {
		public static let md5sum: String = "ef145a45a5f47b77b7f5cdde4b16c942"
		public static let datatype = "trajectory_msgs/MultiDOFJointTrajectory"
		public static let definition = """
			# The header is used to specify the coordinate frame and the reference time for the trajectory durations
			Header header
			# A representation of a multi-dof joint trajectory (each point is a transformation)
			# Each point along the trajectory will include an array of positions/velocities/accelerations
			# that has the same length as the array of joint names, and has the same order of joints as 
			# the joint names array.
			string[] joint_names
			MultiDOFJointTrajectoryPoint[] points
			"""

		public static let hasHeader = true

	
		public var header: std_msgs.Header
		public var joint_names: [String]
		public var points: [MultiDOFJointTrajectoryPoint]

		public init(header: std_msgs.Header, joint_names: [String], points: [MultiDOFJointTrajectoryPoint]) {
			self.header = header
			self.joint_names = joint_names
			self.points = points
		}

		public init() {
			header = std_msgs.Header()
			joint_names = [String]()
			points = [MultiDOFJointTrajectoryPoint]()
		}
	}
}