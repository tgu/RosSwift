// Generated by msgbuilder 2020-05-14 06:44:12 +0000

import StdMsgs

extension std_srvs {
	public enum Trigger: ServiceProt {
		public static let md5sum: String = "937c9679a518e3a18d831e57125ea522"
		public static let datatype = "std_srvs/Trigger"

	
		public struct Request: ServiceRequestMessage {
			public static let md5sum: String = "d41d8cd98f00b204e9800998ecf8427e"
			public static let datatype = "std_srvs/TriggerRequest"
			public typealias ServiceType = Trigger
			public static let definition = """
				
				"""
	
		
		
	
	
	
			public init() {
		
			}
		}

	
		public struct Response: ServiceResponseMessage {
			public static let md5sum: String = "937c9679a518e3a18d831e57125ea522"
			public static let datatype = "std_srvs/TriggerResponse"
			public typealias ServiceType = Trigger
			public static let definition = """
				bool success   # indicate successful run of triggered service
				string message # informational, e.g. for error messages
				"""
	
		
			public var success: Bool
			public var message: String
	
			public init(success: Bool, message: String) {
				self.success = success
				self.message = message
			}
	
			public init() {
				success = Bool()
				message = String()
			}
		}

	}
}