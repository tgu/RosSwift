import Foundation
import StdMsgs
import RosTime


extension diagnostic_msgs {
	/// This message holds the status of an individual component of the robot.
	/// 
	/// Possible levels of operations
	public struct DiagnosticStatus: Message {
		public static let md5sum: String = "405b1ea96476bc1e09e311f0c44f9a93"
		public static let datatype = "diagnostic_msgs/DiagnosticStatus"
		public static let definition = """
			# This message holds the status of an individual component of the robot.
			# 
			# Possible levels of operations
			byte OK=0
			byte WARN=1
			byte ERROR=2
			byte STALE=3
			byte level # level of operation enumerated above 
			string name # a description of the test/component reporting
			string message # a description of the status
			string hardware_id # a hardware unique string
			KeyValue[] values # an array of values associated with the status
			"""

		public static let hasHeader = false

		public static let OK: Int8 = 0
		public static let WARN: Int8 = 1
		public static let ERROR: Int8 = 2
		public static let STALE: Int8 = 3
		public var level: Int8
		public var name: String
		public var message: String
		public var hardware_id: String
		public var values: [KeyValue]

		public init(level: Int8, name: String, message: String, hardware_id: String, values: [KeyValue]) {
			self.level = level
			self.name = name
			self.message = message
			self.hardware_id = hardware_id
			self.values = values
		}

		public init() {
			level = Int8()
			name = String()
			message = String()
			hardware_id = String()
			values = [KeyValue]()
		}
	}
}