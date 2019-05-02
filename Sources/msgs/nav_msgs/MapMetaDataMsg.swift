import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension nav_msgs {
	/// This hold basic information about the characterists of the OccupancyGrid
	/// The time at which the map was loaded
	/// The map resolution [m/cell]
	/// Map width [cells]
	/// Map height [cells]
	/// The origin of the map [m, m, rad].  This is the real-world pose of the
	/// cell (0,0) in the map.
	public struct MapMetaData: Message {
		public static let md5sum: String = "10cfc8a2818024d3248802c00c95f11b"
		public static let datatype = "nav_msgs/MapMetaData"
		public static let definition = """
			# This hold basic information about the characterists of the OccupancyGrid
			# The time at which the map was loaded
			time map_load_time
			# The map resolution [m/cell]
			float32 resolution
			# Map width [cells]
			uint32 width
			# Map height [cells]
			uint32 height
			# The origin of the map [m, m, rad].  This is the real-world pose of the
			# cell (0,0) in the map.
			geometry_msgs/Pose origin
			"""

		public static let hasHeader = false

	
		public var map_load_time: Time
		public var resolution: Float32
		public var width: UInt32
		public var height: UInt32
		public var origin: geometry_msgs.Pose

		public init(map_load_time: Time, resolution: Float32, width: UInt32, height: UInt32, origin: geometry_msgs.Pose) {
			self.map_load_time = map_load_time
			self.resolution = resolution
			self.width = width
			self.height = height
			self.origin = origin
		}

		public init() {
			map_load_time = Time()
			resolution = Float32()
			width = UInt32()
			height = UInt32()
			origin = geometry_msgs.Pose()
		}
	}
}