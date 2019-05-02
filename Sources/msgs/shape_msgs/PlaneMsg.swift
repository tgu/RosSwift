import Foundation
import StdMsgs
import RosTime


extension shape_msgs {
	/// Representation of a plane, using the plane equation ax + by + cz + d = 0
	/// a := coef[0]
	/// b := coef[1]
	/// c := coef[2]
	/// d := coef[3]
	public struct Plane: Message {
		public static let md5sum: String = "2c1b92ed8f31492f8e73f6a4a44ca796"
		public static let datatype = "shape_msgs/Plane"
		public static let definition = """
			# Representation of a plane, using the plane equation ax + by + cz + d = 0
			# a := coef[0]
			# b := coef[1]
			# c := coef[2]
			# d := coef[3]
			float64[4] coef
			"""

		public static let hasHeader = false

	
		public var coef: [Float64]

		public init(coef: [Float64]) {
			assert(coef.count == 4)
			self.coef = coef
		}

		public init() {
			coef = [Float64](repeating: 0, count: 4)
		}
	}
}