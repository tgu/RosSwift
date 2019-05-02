import Foundation
import StdMsgs
import RosTime


extension control_msgs {

	public struct PidState: Message {
		public static let md5sum: String = "b138ec00e886c10e73f27e8712252ea6"
		public static let datatype = "control_msgs/PidState"
		public static let definition = """
			Header header
			duration timestep
			float64 error
			float64 error_dot
			float64 p_error
			float64 i_error
			float64 d_error
			float64 p_term
			float64 i_term
			float64 d_term
			float64 i_max
			float64 i_min
			float64 output
			"""

		public static let hasHeader = false

	
		public var header: std_msgs.Header
		public var timestep: Duration
		public var error: Float64
		public var error_dot: Float64
		public var p_error: Float64
		public var i_error: Float64
		public var d_error: Float64
		public var p_term: Float64
		public var i_term: Float64
		public var d_term: Float64
		public var i_max: Float64
		public var i_min: Float64
		public var output: Float64

		public init(header: std_msgs.Header, timestep: Duration, error: Float64, error_dot: Float64, p_error: Float64, i_error: Float64, d_error: Float64, p_term: Float64, i_term: Float64, d_term: Float64, i_max: Float64, i_min: Float64, output: Float64) {
			self.header = header
			self.timestep = timestep
			self.error = error
			self.error_dot = error_dot
			self.p_error = p_error
			self.i_error = i_error
			self.d_error = d_error
			self.p_term = p_term
			self.i_term = i_term
			self.d_term = d_term
			self.i_max = i_max
			self.i_min = i_min
			self.output = output
		}

		public init() {
			header = std_msgs.Header()
			timestep = Duration()
			error = Float64()
			error_dot = Float64()
			p_error = Float64()
			i_error = Float64()
			d_error = Float64()
			p_term = Float64()
			i_term = Float64()
			d_term = Float64()
			i_max = Float64()
			i_min = Float64()
			output = Float64()
		}
	}
}