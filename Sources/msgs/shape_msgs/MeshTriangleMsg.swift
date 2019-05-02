import Foundation
import StdMsgs
import RosTime


extension shape_msgs {
	/// Definition of a triangle's vertices
	public struct MeshTriangle: Message {
		public static let md5sum: String = "23688b2e6d2de3d32fe8af104a903253"
		public static let datatype = "shape_msgs/MeshTriangle"
		public static let definition = """
			# Definition of a triangle's vertices
			uint32[3] vertex_indices
			"""

		public static let hasHeader = false

	
		public var vertex_indices: [UInt32]

		public init(vertex_indices: [UInt32]) {
			assert(vertex_indices.count == 3)
			self.vertex_indices = vertex_indices
		}

		public init() {
			vertex_indices = [UInt32](repeating: 0, count: 3)
		}
	}
}