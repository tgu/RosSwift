import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension visualization_msgs {
	/// See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz
	///Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
	///Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
	///number of colors must either be 0 or equal to the number of points
	///NOTE: alpha is not yet used
	/// NOTE: only used for text markers
	/// NOTE: only used for MESH_RESOURCE markers
	public struct Marker: Message {
		public static let md5sum: String = "fc60f67ee1b0328d53f32573aafeb4d9"
		public static let datatype = "visualization_msgs/Marker"
		public static let definition = """
			# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz
			uint8 ARROW=0
			uint8 CUBE=1
			uint8 SPHERE=2
			uint8 CYLINDER=3
			uint8 LINE_STRIP=4
			uint8 LINE_LIST=5
			uint8 CUBE_LIST=6
			uint8 SPHERE_LIST=7
			uint8 POINTS=8
			uint8 TEXT_VIEW_FACING=9
			uint8 MESH_RESOURCE=10
			uint8 TRIANGLE_LIST=11
			uint8 ADD=0
			uint8 MODIFY=0
			uint8 DELETE=2
			uint8 DELETEALL=3
			Header header                        # header for time/frame information
			string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
			int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
			int32 type 		                       # Type of object
			int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
			geometry_msgs/Pose pose                 # Pose of the object
			geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
			std_msgs/ColorRGBA color             # Color [0.0-1.0]
			duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
			bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep
			#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
			geometry_msgs/Point[] points
			#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
			#number of colors must either be 0 or equal to the number of points
			#NOTE: alpha is not yet used
			std_msgs/ColorRGBA[] colors
			# NOTE: only used for text markers
			string text
			# NOTE: only used for MESH_RESOURCE markers
			string mesh_resource
			bool mesh_use_embedded_materials
			"""

		public static let hasHeader = true

		public static let ARROW: UInt8 = 0
		public static let CUBE: UInt8 = 1
		public static let SPHERE: UInt8 = 2
		public static let CYLINDER: UInt8 = 3
		public static let LINE_STRIP: UInt8 = 4
		public static let LINE_LIST: UInt8 = 5
		public static let CUBE_LIST: UInt8 = 6
		public static let SPHERE_LIST: UInt8 = 7
		public static let POINTS: UInt8 = 8
		public static let TEXT_VIEW_FACING: UInt8 = 9
		public static let MESH_RESOURCE: UInt8 = 10
		public static let TRIANGLE_LIST: UInt8 = 11
		public static let ADD: UInt8 = 0
		public static let MODIFY: UInt8 = 0
		public static let DELETE: UInt8 = 2
		public static let DELETEALL: UInt8 = 3
		public var header: std_msgs.Header
		public var ns: String
		public var id: Int32
		public var type: Int32
		public var action: Int32
		public var pose: geometry_msgs.Pose
		public var scale: geometry_msgs.Vector3
		public var color: std_msgs.ColorRGBA
		public var lifetime: Duration
		public var frame_locked: Bool
		public var points: [geometry_msgs.Point]
		public var colors: [std_msgs.ColorRGBA]
		public var text: String
		public var mesh_resource: String
		public var mesh_use_embedded_materials: Bool

		public init(header: std_msgs.Header, ns: String, id: Int32, type: Int32, action: Int32, pose: geometry_msgs.Pose, scale: geometry_msgs.Vector3, color: std_msgs.ColorRGBA, lifetime: Duration, frame_locked: Bool, points: [geometry_msgs.Point], colors: [std_msgs.ColorRGBA], text: String, mesh_resource: String, mesh_use_embedded_materials: Bool) {
			self.header = header
			self.ns = ns
			self.id = id
			self.type = type
			self.action = action
			self.pose = pose
			self.scale = scale
			self.color = color
			self.lifetime = lifetime
			self.frame_locked = frame_locked
			self.points = points
			self.colors = colors
			self.text = text
			self.mesh_resource = mesh_resource
			self.mesh_use_embedded_materials = mesh_use_embedded_materials
		}

		public init() {
			header = std_msgs.Header()
			ns = String()
			id = Int32()
			type = Int32()
			action = Int32()
			pose = geometry_msgs.Pose()
			scale = geometry_msgs.Vector3()
			color = std_msgs.ColorRGBA()
			lifetime = Duration()
			frame_locked = Bool()
			points = [geometry_msgs.Point]()
			colors = [std_msgs.ColorRGBA]()
			text = String()
			mesh_resource = String()
			mesh_use_embedded_materials = Bool()
		}
	}
}