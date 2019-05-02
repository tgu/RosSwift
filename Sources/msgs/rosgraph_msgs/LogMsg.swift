import Foundation
import StdMsgs
import RosTime


extension rosgraph_msgs {
	///
	/// Severity level constants
	///
	///
	/// Fields
	///
	public struct Log: Message {
		public static var md5sum: String = "acffd30cd6b6de30f120938c17c593fb"
		public static var datatype = "rosgraph_msgs/Log"
		public static var definition = """
			##
			## Severity level constants
			##
			byte DEBUG=1 #debug level
			byte INFO=2  #general level
			byte WARN=4  #warning level
			byte ERROR=8 #error level
			byte FATAL=16 #fatal/critical level
			##
			## Fields
			##
			Header header
			byte level
			string name # name of the node
			string msg # message 
			string file # file the message came from
			string function # function the message came from
			uint32 line # line the message came from
			string[] topics # topic names that the node publishes
			"""

		public static let hasHeader = true

		public static let DEBUG: Int8 = 1
		public static let INFO: Int8 = 2
		public static let WARN: Int8 = 4
		public static let ERROR: Int8 = 8
		public static let FATAL: Int8 = 16
		public var header: std_msgs.header
		public var level: Int8
		public var name: String
		public var msg: String
		public var file: String
		public var function: String
		public var line: UInt32
		public var topics: [String]

		public init(header: std_msgs.header, level: Int8, name: String, msg: String, file: String, function: String, line: UInt32, topics: [String]) {
			self.header = header
			self.level = level
			self.name = name
			self.msg = msg
			self.file = file
			self.function = function
			self.line = line
			self.topics = topics
		}

		public init() {
			header = std_msgs.header()
			level = Int8()
			name = String()
			msg = String()
			file = String()
			function = String()
			line = UInt32()
			topics = [String]()
		}
	}
}