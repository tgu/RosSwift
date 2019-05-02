import Foundation
import StdMsgs
import RosTime


extension visualization_msgs {
	/// Identifying string. Must be unique in the topic namespace
	/// that this server works on.
	/// Sequence number.
	/// The client will use this to detect if it has missed an update.
	/// Type holds the purpose of this message.  It must be one of UPDATE or KEEP_ALIVE.
	/// UPDATE: Incremental update to previous state. 
	///         The sequence number must be 1 higher than for
	///         the previous update.
	/// KEEP_ALIVE: Indicates the that the server is still living.
	///             The sequence number does not increase.
	///             No payload data should be filled out (markers, poses, or erases).
	///Note: No guarantees on the order of processing.
	///      Contents must be kept consistent by sender.
	///Markers to be added or updated
	///Poses of markers that should be moved
	///Names of markers to be erased
	public struct InteractiveMarkerUpdate: Message {
		public static let md5sum: String = "4ddf59c78da88023813ed0e3c3d68214"
		public static let datatype = "visualization_msgs/InteractiveMarkerUpdate"
		public static let definition = """
			# Identifying string. Must be unique in the topic namespace
			# that this server works on.
			string server_id
			# Sequence number.
			# The client will use this to detect if it has missed an update.
			uint64 seq_num
			# Type holds the purpose of this message.  It must be one of UPDATE or KEEP_ALIVE.
			# UPDATE: Incremental update to previous state. 
			#         The sequence number must be 1 higher than for
			#         the previous update.
			# KEEP_ALIVE: Indicates the that the server is still living.
			#             The sequence number does not increase.
			#             No payload data should be filled out (markers, poses, or erases).
			uint8 KEEP_ALIVE = 0
			uint8 UPDATE = 1
			uint8 type
			#Note: No guarantees on the order of processing.
			#      Contents must be kept consistent by sender.
			#Markers to be added or updated
			InteractiveMarker[] markers
			#Poses of markers that should be moved
			InteractiveMarkerPose[] poses
			#Names of markers to be erased
			string[] erases
			"""

		public static let hasHeader = false

		public static let KEEP_ALIVE : UInt8 = 0
		public static let UPDATE : UInt8 = 1
		public var server_id: String
		public var seq_num: UInt64
		public var type: UInt8
		public var markers: [InteractiveMarker]
		public var poses: [InteractiveMarkerPose]
		public var erases: [String]

		public init(server_id: String, seq_num: UInt64, type: UInt8, markers: [InteractiveMarker], poses: [InteractiveMarkerPose], erases: [String]) {
			self.server_id = server_id
			self.seq_num = seq_num
			self.type = type
			self.markers = markers
			self.poses = poses
			self.erases = erases
		}

		public init() {
			server_id = String()
			seq_num = UInt64()
			type = UInt8()
			markers = [InteractiveMarker]()
			poses = [InteractiveMarkerPose]()
			erases = [String]()
		}
	}
}