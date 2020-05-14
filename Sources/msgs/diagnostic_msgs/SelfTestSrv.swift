// Generated by msgbuilder 2020-05-14 06:44:12 +0000

import StdMsgs

extension diagnostic_msgs {
	public enum SelfTest: ServiceProt {
		public static let md5sum: String = "36017f81f034610a27eda23327c2767c"
		public static let datatype = "diagnostic_msgs/SelfTest"

	
		public struct Request: ServiceRequestMessage {
			public static let md5sum: String = "d41d8cd98f00b204e9800998ecf8427e"
			public static let datatype = "diagnostic_msgs/SelfTestRequest"
			public typealias ServiceType = SelfTest
			public static let definition = """
				
				"""
	
		
		
	
	
	
			public init() {
		
			}
		}

	
		public struct Response: ServiceResponseMessage {
			public static let md5sum: String = "36017f81f034610a27eda23327c2767c"
			public static let datatype = "diagnostic_msgs/SelfTestResponse"
			public typealias ServiceType = SelfTest
			public static let definition = """
				string id
				byte passed
				DiagnosticStatus[] status
				"""
	
		
			public var id: String
			public var passed: Int8
			public var status: [DiagnosticStatus]
	
			public init(id: String, passed: Int8, status: [DiagnosticStatus]) {
				self.id = id
				self.passed = passed
				self.status = status
			}
	
			public init() {
				id = String()
				passed = Int8()
				status = [DiagnosticStatus]()
			}
		}

	}
}