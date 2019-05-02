import Foundation
import StdMsgs
import RosTime


extension nav_msgs {
	/// This represents a 2-D grid map, in which each cell represents the probability of
	/// occupancy.
	///MetaData for the map
	/// The map data, in row-major order, starting with (0,0).  Occupancy
	/// probabilities are in the range [0,100].  Unknown is -1.
	public struct OccupancyGrid: Message {
		public static let md5sum: String = "3381f2d731d4076ec5c71b0759edbe4e"
		public static let datatype = "nav_msgs/OccupancyGrid"
		public static let definition = """
			# This represents a 2-D grid map, in which each cell represents the probability of
			# occupancy.
			Header header 
			#MetaData for the map
			MapMetaData info
			# The map data, in row-major order, starting with (0,0).  Occupancy
			# probabilities are in the range [0,100].  Unknown is -1.
			int8[] data
			"""

		public static let hasHeader = true

	
		public var header: std_msgs.Header
		public var info: MapMetaData
		public var data: [Int8]

		public init(header: std_msgs.Header, info: MapMetaData, data: [Int8]) {
			self.header = header
			self.info = info
			self.data = data
		}

		public init() {
			header = std_msgs.Header()
			info = MapMetaData()
			data = [Int8]()
		}
	}
}