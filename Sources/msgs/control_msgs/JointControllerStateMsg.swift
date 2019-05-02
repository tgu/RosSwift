import Foundation
import StdMsgs
import RosTime


extension control_msgs {

	public struct JointControllerState: Message {
		public static let md5sum: String = "987ad85e4756f3aef7f1e5e7fe0595d1"
		public static let datatype = "control_msgs/JointControllerState"
		public static let definition = """
			Header header
			float64 set_point
			float64 process_value
			float64 process_value_dot
			float64 error
			float64 time_step
			float64 command
			float64 p
			float64 i
			float64 d
			float64 i_clamp
			bool antiwindup
			"""

		public static let hasHeader = false

	
		public var header: std_msgs.Header
		public var set_point: Float64
		public var process_value: Float64
		public var process_value_dot: Float64
		public var error: Float64
		public var time_step: Float64
		public var command: Float64
		public var p: Float64
		public var i: Float64
		public var d: Float64
		public var i_clamp: Float64
		public var antiwindup: Bool

		public init(header: std_msgs.Header, set_point: Float64, process_value: Float64, process_value_dot: Float64, error: Float64, time_step: Float64, command: Float64, p: Float64, i: Float64, d: Float64, i_clamp: Float64, antiwindup: Bool) {
			self.header = header
			self.set_point = set_point
			self.process_value = process_value
			self.process_value_dot = process_value_dot
			self.error = error
			self.time_step = time_step
			self.command = command
			self.p = p
			self.i = i
			self.d = d
			self.i_clamp = i_clamp
			self.antiwindup = antiwindup
		}

		public init() {
			header = std_msgs.Header()
			set_point = Float64()
			process_value = Float64()
			process_value_dot = Float64()
			error = Float64()
			time_step = Float64()
			command = Float64()
			p = Float64()
			i = Float64()
			d = Float64()
			i_clamp = Float64()
			antiwindup = Bool()
		}
	}
}