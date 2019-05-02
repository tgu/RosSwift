import Foundation
import StdMsgs
import RosTime


extension control_msgs {
	/// The tolerances specify the amount the position, velocity, and
	/// accelerations can vary from the setpoints.  For example, in the case
	/// of trajectory control, when the actual position varies beyond
	/// (desired position + position tolerance), the trajectory goal may
	/// abort.
	/// 
	/// There are two special values for tolerances:
	///  * 0 - The tolerance is unspecified and will remain at whatever the default is
	///  * -1 - The tolerance is "erased".  If there was a default, the joint will be
	///         allowed to move without restriction.
	public struct JointTolerance: Message {
		public static let md5sum: String = "f544fe9c16cf04547e135dd6063ff5be"
		public static let datatype = "control_msgs/JointTolerance"
		public static let definition = """
			# The tolerances specify the amount the position, velocity, and
			# accelerations can vary from the setpoints.  For example, in the case
			# of trajectory control, when the actual position varies beyond
			# (desired position + position tolerance), the trajectory goal may
			# abort.
			# 
			# There are two special values for tolerances:
			#  * 0 - The tolerance is unspecified and will remain at whatever the default is
			#  * -1 - The tolerance is "erased".  If there was a default, the joint will be
			#         allowed to move without restriction.
			string name
			float64 position  # in radians or meters (for a revolute or prismatic joint, respectively)
			float64 velocity  # in rad/sec or m/sec
			float64 acceleration  # in rad/sec^2 or m/sec^2
			"""

		public static let hasHeader = false

	
		public var name: String
		public var position: Float64
		public var velocity: Float64
		public var acceleration: Float64

		public init(name: String, position: Float64, velocity: Float64, acceleration: Float64) {
			self.name = name
			self.position = position
			self.velocity = velocity
			self.acceleration = acceleration
		}

		public init() {
			name = String()
			position = Float64()
			velocity = Float64()
			acceleration = Float64()
		}
	}
}