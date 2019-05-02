import Foundation
import StdMsgs
import RosTime


extension control_msgs {
	/// Used in time-stamping the message.
	/// Name list of the joints. You don't need to specify all joints of the
	/// robot. Joint names are case-sensitive.
	/// A position command to the joints listed in joint_names.
	/// The order must be identical.
	/// Units are meters or radians.
	/// If displacements and velocities are filled, a profiled motion is requested.
	/// A velocity command to the joints listed in joint_names.
	/// The order must be identical.
	/// Units are m/s or rad/s.
	/// If displacements and velocities are filled, a profiled motion is requested.
	public struct JointJog: Message {
		public static let md5sum: String = "1685da700c8c2e1254afc92a5fb89c96"
		public static let datatype = "control_msgs/JointJog"
		public static let definition = """
			# Used in time-stamping the message.
			Header header
			# Name list of the joints. You don't need to specify all joints of the
			# robot. Joint names are case-sensitive.
			string[] joint_names
			# A position command to the joints listed in joint_names.
			# The order must be identical.
			# Units are meters or radians.
			# If displacements and velocities are filled, a profiled motion is requested.
			float64[] displacements # or position_deltas
			# A velocity command to the joints listed in joint_names.
			# The order must be identical.
			# Units are m/s or rad/s.
			# If displacements and velocities are filled, a profiled motion is requested.
			float64[] velocities
			float64 duration
			"""

		public static let hasHeader = false

	
		public var header: std_msgs.Header
		public var joint_names: [String]
		public var displacements: [Float64]
		public var velocities: [Float64]
		public var duration: Float64

		public init(header: std_msgs.Header, joint_names: [String], displacements: [Float64], velocities: [Float64], duration: Float64) {
			self.header = header
			self.joint_names = joint_names
			self.displacements = displacements
			self.velocities = velocities
			self.duration = duration
		}

		public init() {
			header = std_msgs.Header()
			joint_names = [String]()
			displacements = [Float64]()
			velocities = [Float64]()
			duration = Float64()
		}
	}
}