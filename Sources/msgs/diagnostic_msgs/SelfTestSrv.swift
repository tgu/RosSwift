// Generated by msgbuilder 2020-05-12 11:56:11 +0000

import StdMsgs

extension diagnostic_msgs {
	public struct SelfTest: ServiceProt {
		public typealias Request = SelfTestRequest
		public typealias Response = SelfTestResponse
		public var request: Request
		public var response: Response

		public static let md5sum: String = "36017f81f034610a27eda23327c2767c"
		public static let datatype = "diagnostic_msgs/SelfTest"
	}
}