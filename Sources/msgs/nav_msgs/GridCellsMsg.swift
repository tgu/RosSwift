import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension nav_msgs {
	///an array of cells in a 2D grid
	public struct GridCells: Message {
		public static let md5sum: String = "b9e4f5df6d28e272ebde00a3994830f5"
		public static let datatype = "nav_msgs/GridCells"
		public static let definition = """
			#an array of cells in a 2D grid
			Header header
			float32 cell_width
			float32 cell_height
			geometry_msgs/Point[] cells
			"""

		public static let hasHeader = true

	
		public var header: std_msgs.Header
		public var cell_width: Float32
		public var cell_height: Float32
		public var cells: [geometry_msgs.Point]

		public init(header: std_msgs.Header, cell_width: Float32, cell_height: Float32, cells: [geometry_msgs.Point]) {
			self.header = header
			self.cell_width = cell_width
			self.cell_height = cell_height
			self.cells = cells
		}

		public init() {
			header = std_msgs.Header()
			cell_width = Float32()
			cell_height = Float32()
			cells = [geometry_msgs.Point]()
		}
	}
}