import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension visualization_msgs {
	/// Time/frame info.
	/// If header.time is set to 0, the marker will be retransformed into
	/// its frame on each timestep. You will receive the pose feedback
	/// in the same frame.
	/// Otherwise, you might receive feedback in a different frame.
	/// For rviz, this will be the current 'fixed frame' set by the user.
	/// Initial pose. Also, defines the pivot point for rotations.
	/// Identifying string. Must be globally unique in
	/// the topic that this message is sent through.
	/// Short description (< 40 characters).
	/// Scale to be used for default controls (default=1).
	/// All menu and submenu entries associated with this marker.
	/// List of controls displayed for this marker.
	public struct InteractiveMarker: Message {
		public static let md5sum: String = "83dae4e7bbe99e8aee7cc47d6b4b64a4"
		public static let datatype = "visualization_msgs/InteractiveMarker"
		public static let definition = """
			# Time/frame info.
			# If header.time is set to 0, the marker will be retransformed into
			# its frame on each timestep. You will receive the pose feedback
			# in the same frame.
			# Otherwise, you might receive feedback in a different frame.
			# For rviz, this will be the current 'fixed frame' set by the user.
			Header header
			# Initial pose. Also, defines the pivot point for rotations.
			geometry_msgs/Pose pose
			# Identifying string. Must be globally unique in
			# the topic that this message is sent through.
			string name
			# Short description (< 40 characters).
			string description
			# Scale to be used for default controls (default=1).
			float32 scale
			# All menu and submenu entries associated with this marker.
			MenuEntry[] menu_entries
			# List of controls displayed for this marker.
			InteractiveMarkerControl[] controls
			"""

		public static let hasHeader = true

	
		public var header: std_msgs.Header
		public var pose: geometry_msgs.Pose
		public var name: String
		public var description: String
		public var scale: Float32
		public var menu_entries: [MenuEntry]
		public var controls: [InteractiveMarkerControl]

		public init(header: std_msgs.Header, pose: geometry_msgs.Pose, name: String, description: String, scale: Float32, menu_entries: [MenuEntry], controls: [InteractiveMarkerControl]) {
			self.header = header
			self.pose = pose
			self.name = name
			self.description = description
			self.scale = scale
			self.menu_entries = menu_entries
			self.controls = controls
		}

		public init() {
			header = std_msgs.Header()
			pose = geometry_msgs.Pose()
			name = String()
			description = String()
			scale = Float32()
			menu_entries = [MenuEntry]()
			controls = [InteractiveMarkerControl]()
		}
	}
}