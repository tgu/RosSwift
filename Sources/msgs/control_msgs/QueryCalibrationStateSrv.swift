// Generated by msgbuilder 2020-05-12 11:56:10 +0000

import StdMsgs

extension control_msgs {
	public struct QueryCalibrationState: ServiceProt {
		public typealias Request = QueryCalibrationStateRequest
		public typealias Response = QueryCalibrationStateResponse
		public var request: Request
		public var response: Response

		public static let md5sum: String = "28af3beedcb84986b8e470dc5470507d"
		public static let datatype = "control_msgs/QueryCalibrationState"
	}
}