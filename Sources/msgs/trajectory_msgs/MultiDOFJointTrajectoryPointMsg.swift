import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension trajectory_msgs {
	/// Each multi-dof joint can specify a transform (up to 6 DOF)
	/// There can be a velocity specified for the origin of the joint 
	/// There can be an acceleration specified for the origin of the joint 
	public struct MultiDOFJointTrajectoryPoint: Message {
		public static let md5sum: String = "3ebe08d1abd5b65862d50e09430db776"
		public static let datatype = "trajectory_msgs/MultiDOFJointTrajectoryPoint"
		public static let definition = """
			# Each multi-dof joint can specify a transform (up to 6 DOF)
			geometry_msgs/Transform[] transforms
			# There can be a velocity specified for the origin of the joint 
			geometry_msgs/Twist[] velocities
			# There can be an acceleration specified for the origin of the joint 
			geometry_msgs/Twist[] accelerations
			duration time_from_start
			"""

		public static let hasHeader = false

	
		public var transforms: [geometry_msgs.Transform]
		public var velocities: [geometry_msgs.Twist]
		public var accelerations: [geometry_msgs.Twist]
		public var time_from_start: Duration

		public init(transforms: [geometry_msgs.Transform], velocities: [geometry_msgs.Twist], accelerations: [geometry_msgs.Twist], time_from_start: Duration) {
			self.transforms = transforms
			self.velocities = velocities
			self.accelerations = accelerations
			self.time_from_start = time_from_start
		}

		public init() {
			transforms = [geometry_msgs.Transform]()
			velocities = [geometry_msgs.Twist]()
			accelerations = [geometry_msgs.Twist]()
			time_from_start = Duration()
		}
	}
}