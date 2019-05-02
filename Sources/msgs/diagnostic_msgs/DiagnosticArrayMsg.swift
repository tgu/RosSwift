import Foundation
import StdMsgs
import RosTime


extension diagnostic_msgs {
	/// This message is used to send diagnostic information about the state of the robot
	public struct DiagnosticArray: Message {
		public static let md5sum: String = "95fdfe4af25735ef2d61c3a265402fa6"
		public static let datatype = "diagnostic_msgs/DiagnosticArray"
		public static let definition = """
			# This message is used to send diagnostic information about the state of the robot
			Header header #for timestamp
			DiagnosticStatus[] status # an array of components being reported on
			"""

		public static let hasHeader = true

	
		public var header: std_msgs.Header
		public var status: [DiagnosticStatus]

		public init(header: std_msgs.Header, status: [DiagnosticStatus]) {
			self.header = header
			self.status = status
		}

		public init() {
			header = std_msgs.Header()
			status = [DiagnosticStatus]()
		}
	}
}