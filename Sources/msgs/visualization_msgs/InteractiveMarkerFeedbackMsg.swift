import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension visualization_msgs {
	/// Time/frame info.
	/// Identifying string. Must be unique in the topic namespace.
	/// Feedback message sent back from the GUI, e.g.
	/// when the status of an interactive marker was modified by the user.
	/// Specifies which interactive marker and control this message refers to
	/// Type of the event
	/// KEEP_ALIVE: sent while dragging to keep up control of the marker
	/// MENU_SELECT: a menu entry has been selected
	/// BUTTON_CLICK: a button control has been clicked
	/// POSE_UPDATE: the pose has been changed using one of the controls
	/// Current pose of the marker
	/// Note: Has to be valid for all feedback types.
	/// Contains the ID of the selected menu entry
	/// Only valid for MENU_SELECT events.
	/// If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point
	/// may contain the 3 dimensional position of the event on the
	/// control.  If it does, mouse_point_valid will be true.  mouse_point
	/// will be relative to the frame listed in the header.
	public struct InteractiveMarkerFeedback: Message {
		public static let md5sum: String = "20a8efac5801deceda77a588311dc8a7"
		public static let datatype = "visualization_msgs/InteractiveMarkerFeedback"
		public static let definition = """
			# Time/frame info.
			Header header
			# Identifying string. Must be unique in the topic namespace.
			string client_id
			# Feedback message sent back from the GUI, e.g.
			# when the status of an interactive marker was modified by the user.
			# Specifies which interactive marker and control this message refers to
			string marker_name
			string control_name
			# Type of the event
			# KEEP_ALIVE: sent while dragging to keep up control of the marker
			# MENU_SELECT: a menu entry has been selected
			# BUTTON_CLICK: a button control has been clicked
			# POSE_UPDATE: the pose has been changed using one of the controls
			uint8 KEEP_ALIVE = 0
			uint8 POSE_UPDATE = 1
			uint8 MENU_SELECT = 2
			uint8 BUTTON_CLICK = 3
			uint8 MOUSE_DOWN = 4
			uint8 MOUSE_UP = 5
			uint8 event_type
			# Current pose of the marker
			# Note: Has to be valid for all feedback types.
			geometry_msgs/Pose pose
			# Contains the ID of the selected menu entry
			# Only valid for MENU_SELECT events.
			uint32 menu_entry_id
			# If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point
			# may contain the 3 dimensional position of the event on the
			# control.  If it does, mouse_point_valid will be true.  mouse_point
			# will be relative to the frame listed in the header.
			geometry_msgs/Point mouse_point
			bool mouse_point_valid
			"""

		public static let hasHeader = true

		public static let KEEP_ALIVE : UInt8 = 0
		public static let POSE_UPDATE : UInt8 = 1
		public static let MENU_SELECT : UInt8 = 2
		public static let BUTTON_CLICK : UInt8 = 3
		public static let MOUSE_DOWN : UInt8 = 4
		public static let MOUSE_UP : UInt8 = 5
		public var header: std_msgs.Header
		public var client_id: String
		public var marker_name: String
		public var control_name: String
		public var event_type: UInt8
		public var pose: geometry_msgs.Pose
		public var menu_entry_id: UInt32
		public var mouse_point: geometry_msgs.Point
		public var mouse_point_valid: Bool

		public init(header: std_msgs.Header, client_id: String, marker_name: String, control_name: String, event_type: UInt8, pose: geometry_msgs.Pose, menu_entry_id: UInt32, mouse_point: geometry_msgs.Point, mouse_point_valid: Bool) {
			self.header = header
			self.client_id = client_id
			self.marker_name = marker_name
			self.control_name = control_name
			self.event_type = event_type
			self.pose = pose
			self.menu_entry_id = menu_entry_id
			self.mouse_point = mouse_point
			self.mouse_point_valid = mouse_point_valid
		}

		public init() {
			header = std_msgs.Header()
			client_id = String()
			marker_name = String()
			control_name = String()
			event_type = UInt8()
			pose = geometry_msgs.Pose()
			menu_entry_id = UInt32()
			mouse_point = geometry_msgs.Point()
			mouse_point_valid = Bool()
		}
	}
}