import Foundation
import StdMsgs
import RosTime


extension trajectory_msgs {
	/// Each trajectory point specifies either positions[, velocities[, accelerations]]
	/// or positions[, effort] for the trajectory to be executed.
	/// All specified values are in the same order as the joint names in JointTrajectory.msg
	public struct JointTrajectoryPoint: Message {
		public static let md5sum: String = "f3cd1e1c4d320c79d6985c904ae5dcd3"
		public static let datatype = "trajectory_msgs/JointTrajectoryPoint"
		public static let definition = """
			# Each trajectory point specifies either positions[, velocities[, accelerations]]
			# or positions[, effort] for the trajectory to be executed.
			# All specified values are in the same order as the joint names in JointTrajectory.msg
			float64[] positions
			float64[] velocities
			float64[] accelerations
			float64[] effort
			duration time_from_start
			"""

		public static let hasHeader = false

	
		public var positions: [Float64]
		public var velocities: [Float64]
		public var accelerations: [Float64]
		public var effort: [Float64]
		public var time_from_start: Duration

		public init(positions: [Float64], velocities: [Float64], accelerations: [Float64], effort: [Float64], time_from_start: Duration) {
			self.positions = positions
			self.velocities = velocities
			self.accelerations = accelerations
			self.effort = effort
			self.time_from_start = time_from_start
		}

		public init() {
			positions = [Float64]()
			velocities = [Float64]()
			accelerations = [Float64]()
			effort = [Float64]()
			time_from_start = Duration()
		}
	}
}