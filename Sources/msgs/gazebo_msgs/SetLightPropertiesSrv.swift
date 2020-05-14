// Generated by msgbuilder 2020-05-14 06:44:12 +0000

import StdMsgs

extension gazebo_msgs {
	public enum SetLightProperties: ServiceProt {
		public static let md5sum: String = "cd58c48ac21e5165abf13bcaa9c079b4"
		public static let datatype = "gazebo_msgs/SetLightProperties"

	
		public struct Request: ServiceRequestMessage {
			public static let md5sum: String = "73ad1ac5e9e312ddf7c74f38ad843f34"
			public static let datatype = "gazebo_msgs/SetLightPropertiesRequest"
			public typealias ServiceType = SetLightProperties
			public static let definition = """
				string light_name                    # name of Gazebo Light
				std_msgs/ColorRGBA diffuse           # diffuse color as red, green, blue, alpha
				float64 attenuation_constant
				float64 attenuation_linear
				float64 attenuation_quadratic
				"""
	
		
			public var light_name: String
			public var diffuse: std_msgs.ColorRGBA
			public var attenuation_constant: Float64
			public var attenuation_linear: Float64
			public var attenuation_quadratic: Float64
	
			public init(light_name: String, diffuse: std_msgs.ColorRGBA, attenuation_constant: Float64, attenuation_linear: Float64, attenuation_quadratic: Float64) {
				self.light_name = light_name
				self.diffuse = diffuse
				self.attenuation_constant = attenuation_constant
				self.attenuation_linear = attenuation_linear
				self.attenuation_quadratic = attenuation_quadratic
			}
	
			public init() {
				light_name = String()
				diffuse = std_msgs.ColorRGBA()
				attenuation_constant = Float64()
				attenuation_linear = Float64()
				attenuation_quadratic = Float64()
			}
		}

	
		public struct Response: ServiceResponseMessage {
			public static let md5sum: String = "2ec6f3eff0161f4257b808b12bc830c2"
			public static let datatype = "gazebo_msgs/SetLightPropertiesResponse"
			public typealias ServiceType = SetLightProperties
			public static let definition = """
				bool success                         # return true if get successful
				string status_message                # comments if available
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