import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension visualization_msgs {
	/// Represents a control that is to be displayed together with an interactive marker
	/// Identifying string for this control.
	/// You need to assign a unique value to this to receive feedback from the GUI
	/// on what actions the user performs on this control (e.g. a button click).
	/// Defines the local coordinate frame (relative to the pose of the parent
	/// interactive marker) in which is being rotated and translated.
	/// Default: Identity
	/// Orientation mode: controls how orientation changes.
	/// INHERIT: Follow orientation of interactive marker
	/// FIXED: Keep orientation fixed at initial state
	/// VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
	/// Interaction mode for this control
	/// 
	/// NONE: This control is only meant for visualization; no context menu.
	/// MENU: Like NONE, but right-click menu is active.
	/// BUTTON: Element can be left-clicked.
	/// MOVE_AXIS: Translate along local x-axis.
	/// MOVE_PLANE: Translate in local y-z plane.
	/// ROTATE_AXIS: Rotate around local x-axis.
	/// MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
	/// "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
	/// MOVE_3D: Translate freely in 3D space.
	/// ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
	/// MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
	/// If true, the contained markers will also be visible
	/// when the gui is not in interactive mode.
	/// Markers to be displayed as custom visual representation.
	/// Leave this empty to use the default control handles.
	///
	/// Note: 
	/// - The markers can be defined in an arbitrary coordinate frame,
	///   but will be transformed into the local frame of the interactive marker.
	/// - If the header of a marker is empty, its pose will be interpreted as 
	///   relative to the pose of the parent interactive marker.
	/// In VIEW_FACING mode, set this to true if you don't want the markers
	/// to be aligned with the camera view point. The markers will show up
	/// as in INHERIT mode.
	/// Short description (< 40 characters) of what this control does,
	/// e.g. "Move the robot". 
	/// Default: A generic description based on the interaction mode
	public struct InteractiveMarkerControl: Message {
		public static let md5sum: String = "c5ce59f365dac3ae54296152e665de0b"
		public static let datatype = "visualization_msgs/InteractiveMarkerControl"
		public static let definition = """
			# Represents a control that is to be displayed together with an interactive marker
			# Identifying string for this control.
			# You need to assign a unique value to this to receive feedback from the GUI
			# on what actions the user performs on this control (e.g. a button click).
			string name
			# Defines the local coordinate frame (relative to the pose of the parent
			# interactive marker) in which is being rotated and translated.
			# Default: Identity
			geometry_msgs/Quaternion orientation
			# Orientation mode: controls how orientation changes.
			# INHERIT: Follow orientation of interactive marker
			# FIXED: Keep orientation fixed at initial state
			# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
			uint8 INHERIT = 0 
			uint8 FIXED = 1
			uint8 VIEW_FACING = 2
			uint8 orientation_mode
			# Interaction mode for this control
			# 
			# NONE: This control is only meant for visualization; no context menu.
			# MENU: Like NONE, but right-click menu is active.
			# BUTTON: Element can be left-clicked.
			# MOVE_AXIS: Translate along local x-axis.
			# MOVE_PLANE: Translate in local y-z plane.
			# ROTATE_AXIS: Rotate around local x-axis.
			# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
			uint8 NONE = 0 
			uint8 MENU = 1
			uint8 BUTTON = 2
			uint8 MOVE_AXIS = 3 
			uint8 MOVE_PLANE = 4
			uint8 ROTATE_AXIS = 5
			uint8 MOVE_ROTATE = 6
			# "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
			# MOVE_3D: Translate freely in 3D space.
			# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
			# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
			uint8 MOVE_3D = 7
			uint8 ROTATE_3D = 8
			uint8 MOVE_ROTATE_3D = 9
			uint8 interaction_mode
			# If true, the contained markers will also be visible
			# when the gui is not in interactive mode.
			bool always_visible
			# Markers to be displayed as custom visual representation.
			# Leave this empty to use the default control handles.
			#
			# Note: 
			# - The markers can be defined in an arbitrary coordinate frame,
			#   but will be transformed into the local frame of the interactive marker.
			# - If the header of a marker is empty, its pose will be interpreted as 
			#   relative to the pose of the parent interactive marker.
			Marker[] markers
			# In VIEW_FACING mode, set this to true if you don't want the markers
			# to be aligned with the camera view point. The markers will show up
			# as in INHERIT mode.
			bool independent_marker_orientation
			# Short description (< 40 characters) of what this control does,
			# e.g. "Move the robot". 
			# Default: A generic description based on the interaction mode
			string description
			"""

		public static let hasHeader = false

		public static let INHERIT : UInt8 = 0
		public static let FIXED : UInt8 = 1
		public static let VIEW_FACING : UInt8 = 2
		public static let NONE : UInt8 = 0
		public static let MENU : UInt8 = 1
		public static let BUTTON : UInt8 = 2
		public static let MOVE_AXIS : UInt8 = 3
		public static let MOVE_PLANE : UInt8 = 4
		public static let ROTATE_AXIS : UInt8 = 5
		public static let MOVE_ROTATE : UInt8 = 6
		public static let MOVE_3D : UInt8 = 7
		public static let ROTATE_3D : UInt8 = 8
		public static let MOVE_ROTATE_3D : UInt8 = 9
		public var name: String
		public var orientation: geometry_msgs.Quaternion
		public var orientation_mode: UInt8
		public var interaction_mode: UInt8
		public var always_visible: Bool
		public var markers: [Marker]
		public var independent_marker_orientation: Bool
		public var description: String

		public init(name: String, orientation: geometry_msgs.Quaternion, orientation_mode: UInt8, interaction_mode: UInt8, always_visible: Bool, markers: [Marker], independent_marker_orientation: Bool, description: String) {
			self.name = name
			self.orientation = orientation
			self.orientation_mode = orientation_mode
			self.interaction_mode = interaction_mode
			self.always_visible = always_visible
			self.markers = markers
			self.independent_marker_orientation = independent_marker_orientation
			self.description = description
		}

		public init() {
			name = String()
			orientation = geometry_msgs.Quaternion()
			orientation_mode = UInt8()
			interaction_mode = UInt8()
			always_visible = Bool()
			markers = [Marker]()
			independent_marker_orientation = Bool()
			description = String()
		}
	}
}