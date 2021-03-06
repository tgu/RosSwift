// Generated by msgbuilder 2020-05-15 06:20:49 +0000

import StdMsgs

extension map_msgs {
	public enum GetMapROI: ServiceProt {
		public static let md5sum: String = "81aa75ecf00f4571a9be0d9dc6dea512"
		public static let datatype = "map_msgs/GetMapROI"

	
		public struct Request: ServiceRequestMessage {
			public static let md5sum: String = "43c2ff8f45af555c0eaf070c401e9a47"
			public static let datatype = "map_msgs/GetMapROIRequest"
			public typealias ServiceType = GetMapROI
			public static let definition = """
				float64 x
				float64 y
				float64 l_x
				float64 l_y
				"""
	
			public var x: Float64
			public var y: Float64
			public var l_x: Float64
			public var l_y: Float64
	
			public init(x: Float64, y: Float64, l_x: Float64, l_y: Float64) {
				self.x = x
				self.y = y
				self.l_x = l_x
				self.l_y = l_y
			}
	
			public init() {
				x = Float64()
				y = Float64()
				l_x = Float64()
				l_y = Float64()
			}
		}

	
		public struct Response: ServiceResponseMessage {
			public static let md5sum: String = "4d1986519c00d81967d2891a606b234c"
			public static let datatype = "map_msgs/GetMapROIResponse"
			public typealias ServiceType = GetMapROI
			public static let definition = "nav_msgs/OccupancyGrid sub_map"
	
			public var sub_map: nav_msgs.OccupancyGrid
	
			public init(sub_map: nav_msgs.OccupancyGrid) {
				self.sub_map = sub_map
			}
	
			public init() {
				sub_map = nav_msgs.OccupancyGrid()
			}
		}

	}
}