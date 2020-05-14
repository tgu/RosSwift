// Generated by msgbuilder 2020-05-14 06:44:12 +0000

import StdMsgs

extension gazebo_msgs {
	public enum SetModelConfiguration: ServiceProt {
		public static let md5sum: String = "10e3139d3b669c40afc057d38956fff7"
		public static let datatype = "gazebo_msgs/SetModelConfiguration"

		/// Set joint positions for a model
		public struct Request: ServiceRequestMessage {
			public static let md5sum: String = "160eae60f51fabff255480c70afa289f"
			public static let datatype = "gazebo_msgs/SetModelConfigurationRequest"
			public typealias ServiceType = SetModelConfiguration
			public static let definition = """
				# Set joint positions for a model
				string model_name           # model to set state
				string urdf_param_name      # UNUSED
				string[] joint_names        # list of joints to set positions.  if joint is not listed here, preserve current position.
				float64[] joint_positions   # set to this position.
				"""
	
		
			public var model_name: String
			public var urdf_param_name: String
			public var joint_names: [String]
			public var joint_positions: [Float64]
	
			public init(model_name: String, urdf_param_name: String, joint_names: [String], joint_positions: [Float64]) {
				self.model_name = model_name
				self.urdf_param_name = urdf_param_name
				self.joint_names = joint_names
				self.joint_positions = joint_positions
			}
	
			public init() {
				model_name = String()
				urdf_param_name = String()
				joint_names = [String]()
				joint_positions = [Float64]()
			}
		}

	
		public struct Response: ServiceResponseMessage {
			public static let md5sum: String = "2ec6f3eff0161f4257b808b12bc830c2"
			public static let datatype = "gazebo_msgs/SetModelConfigurationResponse"
			public typealias ServiceType = SetModelConfiguration
			public static let definition = """
				bool success                # return true if setting state successful
				string status_message       # comments if available
				"""
	
		
			public var success: Bool
			public var status_message: String
	
			public init(success: Bool, status_message: String) {
				self.success = success
				self.status_message = status_message
			}
	
			public init() {
				success = Bool()
				status_message = String()
			}
		}

	}
}