import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension visualization_msgs {

	public struct ImageMarker: Message {
		public static let md5sum: String = "1ef0f9f47b1d1b69d41a6b7d703b8a0d"
		public static let datatype = "visualization_msgs/ImageMarker"
		public static let definition = """
			uint8 CIRCLE=0
			uint8 LINE_STRIP=1
			uint8 LINE_LIST=2
			uint8 POLYGON=3
			uint8 POINTS=4
			uint8 ADD=0
			uint8 REMOVE=1
			Header header
			string ns		# namespace, used with id to form a unique id
			int32 id          	# unique id within the namespace
			int32 type        	# CIRCLE/LINE_STRIP/etc.
			int32 action      	# ADD/REMOVE
			geometry_msgs/Point position # 2D, in pixel-coords
			float32 scale	 	# the diameter for a circle, etc.
			std_msgs/ColorRGBA outline_color
			uint8 filled		# whether to fill in the shape with color
			std_msgs/ColorRGBA fill_color # color [0.0-1.0]
			duration lifetime       # How long the object should last before being automatically deleted.  0 means forever
			geometry_msgs/Point[] points # used for LINE_STRIP/LINE_LIST/POINTS/etc., 2D in pixel coords
			std_msgs/ColorRGBA[] outline_colors # a color for each line, point, etc.
			"""

		public static let hasHeader = true

		public static let CIRCLE: UInt8 = 0
		public static let LINE_STRIP: UInt8 = 1
		public static let LINE_LIST: UInt8 = 2
		public static let POLYGON: UInt8 = 3
		public static let POINTS: UInt8 = 4
		public static let ADD: UInt8 = 0
		public static let REMOVE: UInt8 = 1
		public var header: std_msgs.Header
		public var ns: String
		public var id: Int32
		public var type: Int32
		public var action: Int32
		public var position: geometry_msgs.Point
		public var scale: Float32
		public var outline_color: std_msgs.ColorRGBA
		public var filled: UInt8
		public var fill_color: std_msgs.ColorRGBA
		public var lifetime: Duration
		public var points: [geometry_msgs.Point]
		public var outline_colors: [std_msgs.ColorRGBA]

		public init(header: std_msgs.Header, ns: String, id: Int32, type: Int32, action: Int32, position: geometry_msgs.Point, scale: Float32, outline_color: std_msgs.ColorRGBA, filled: UInt8, fill_color: std_msgs.ColorRGBA, lifetime: Duration, points: [geometry_msgs.Point], outline_colors: [std_msgs.ColorRGBA]) {
			self.header = header
			self.ns = ns
			self.id = id
			self.type = type
			self.action = action
			self.position = position
			self.scale = scale
			self.outline_color = outline_color
			self.filled = filled
			self.fill_color = fill_color
			self.lifetime = lifetime
			self.points = points
			self.outline_colors = outline_colors
		}

		public init() {
			header = std_msgs.Header()
			ns = String()
			id = Int32()
			type = Int32()
			action = Int32()
			position = geometry_msgs.Point()
			scale = Float32()
			outline_color = std_msgs.ColorRGBA()
			filled = UInt8()
			fill_color = std_msgs.ColorRGBA()
			lifetime = Duration()
			points = [geometry_msgs.Point]()
			outline_colors = [std_msgs.ColorRGBA]()
		}
	}
}