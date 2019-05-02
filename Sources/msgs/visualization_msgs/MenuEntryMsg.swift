import Foundation
import StdMsgs
import RosTime


extension visualization_msgs {
	/// MenuEntry message.
	/// Each InteractiveMarker message has an array of MenuEntry messages.
	/// A collection of MenuEntries together describe a
	/// menu/submenu/subsubmenu/etc tree, though they are stored in a flat
	/// array.  The tree structure is represented by giving each menu entry
	/// an ID number and a "parent_id" field.  Top-level entries are the
	/// ones with parent_id = 0.  Menu entries are ordered within their
	/// level the same way they are ordered in the containing array.  Parent
	/// entries must appear before their children.
	/// Example:
	/// - id = 3
	///   parent_id = 0
	///   title = "fun"
	/// - id = 2
	///   parent_id = 0
	///   title = "robot"
	/// - id = 4
	///   parent_id = 2
	///   title = "pr2"
	/// - id = 5
	///   parent_id = 2
	///   title = "turtle"
	///
	/// Gives a menu tree like this:
	///  - fun
	///  - robot
	///    - pr2
	///    - turtle
	/// ID is a number for each menu entry.  Must be unique within the
	/// control, and should never be 0.
	/// ID of the parent of this menu entry, if it is a submenu.  If this
	/// menu entry is a top-level entry, set parent_id to 0.
	/// menu / entry title
	/// Arguments to command indicated by command_type (below)
	/// Command_type stores the type of response desired when this menu
	/// entry is clicked.
	/// FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
	/// ROSRUN: execute "rosrun" with arguments given in the command field (above).
	/// ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
	public struct MenuEntry: Message {
		public static let md5sum: String = "1d06a748f24b438282df5310b052f425"
		public static let datatype = "visualization_msgs/MenuEntry"
		public static let definition = """
			# MenuEntry message.
			# Each InteractiveMarker message has an array of MenuEntry messages.
			# A collection of MenuEntries together describe a
			# menu/submenu/subsubmenu/etc tree, though they are stored in a flat
			# array.  The tree structure is represented by giving each menu entry
			# an ID number and a "parent_id" field.  Top-level entries are the
			# ones with parent_id = 0.  Menu entries are ordered within their
			# level the same way they are ordered in the containing array.  Parent
			# entries must appear before their children.
			# Example:
			# - id = 3
			#   parent_id = 0
			#   title = "fun"
			# - id = 2
			#   parent_id = 0
			#   title = "robot"
			# - id = 4
			#   parent_id = 2
			#   title = "pr2"
			# - id = 5
			#   parent_id = 2
			#   title = "turtle"
			#
			# Gives a menu tree like this:
			#  - fun
			#  - robot
			#    - pr2
			#    - turtle
			# ID is a number for each menu entry.  Must be unique within the
			# control, and should never be 0.
			uint32 id
			# ID of the parent of this menu entry, if it is a submenu.  If this
			# menu entry is a top-level entry, set parent_id to 0.
			uint32 parent_id
			# menu / entry title
			string title
			# Arguments to command indicated by command_type (below)
			string command
			# Command_type stores the type of response desired when this menu
			# entry is clicked.
			# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
			# ROSRUN: execute "rosrun" with arguments given in the command field (above).
			# ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
			uint8 FEEDBACK=0
			uint8 ROSRUN=1
			uint8 ROSLAUNCH=2
			uint8 command_type
			"""

		public static let hasHeader = false

		public static let FEEDBACK: UInt8 = 0
		public static let ROSRUN: UInt8 = 1
		public static let ROSLAUNCH: UInt8 = 2
		public var id: UInt32
		public var parent_id: UInt32
		public var title: String
		public var command: String
		public var command_type: UInt8

		public init(id: UInt32, parent_id: UInt32, title: String, command: String, command_type: UInt8) {
			self.id = id
			self.parent_id = parent_id
			self.title = title
			self.command = command
			self.command_type = command_type
		}

		public init() {
			id = UInt32()
			parent_id = UInt32()
			title = String()
			command = String()
			command_type = UInt8()
		}
	}
}