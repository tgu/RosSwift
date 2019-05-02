import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension shape_msgs {
	/// Definition of a mesh
	/// list of triangles; the index values refer to positions in vertices[]
	/// the actual vertices that make up the mesh
	public struct Mesh: Message {
		public static let md5sum: String = "1ffdae9486cd3316a121c578b47a85cc"
		public static let datatype = "shape_msgs/Mesh"
		public static let definition = """
			# Definition of a mesh
			# list of triangles; the index values refer to positions in vertices[]
			MeshTriangle[] triangles
			# the actual vertices that make up the mesh
			geometry_msgs/Point[] vertices
			"""

		public static let hasHeader = false

	
		public var triangles: [MeshTriangle]
		public var vertices: [geometry_msgs.Point]

		public init(triangles: [MeshTriangle], vertices: [geometry_msgs.Point]) {
			self.triangles = triangles
			self.vertices = vertices
		}

		public init() {
			triangles = [MeshTriangle]()
			vertices = [geometry_msgs.Point]()
		}
	}
}