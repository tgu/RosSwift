import Foundation
import StdMsgs
import RosTime


extension visualization_msgs {
	/// Identifying string. Must be unique in the topic namespace
	/// that this server works on.
	/// Sequence number.
	/// The client will use this to detect if it has missed a subsequent
	/// update.  Every update message will have the same sequence number as
	/// an init message.  Clients will likely want to unsubscribe from the
	/// init topic after a successful initialization to avoid receiving
	/// duplicate data.
	/// All markers.
	public struct InteractiveMarkerInit: Message {
		public static let md5sum: String = "8cbf460e330e5f28323b1eba1f10acd0"
		public static let datatype = "visualization_msgs/InteractiveMarkerInit"
		public static let definition = """
			# Identifying string. Must be unique in the topic namespace
			# that this server works on.
			string server_id
			# Sequence number.
			# The client will use this to detect if it has missed a subsequent
			# update.  Every update message will have the same sequence number as
			# an init message.  Clients will likely want to unsubscribe from the
			# init topic after a successful initialization to avoid receiving
			# duplicate data.
			uint64 seq_num
			# All markers.
			InteractiveMarker[] markers
			"""

		public static let hasHeader = false

	
		public var server_id: String
		public var seq_num: UInt64
		public var markers: [InteractiveMarker]

		public init(server_id: String, seq_num: UInt64, markers: [InteractiveMarker]) {
			self.server_id = server_id
			self.seq_num = seq_num
			self.markers = markers
		}

		public init() {
			server_id = String()
			seq_num = UInt64()
			markers = [InteractiveMarker]()
		}
	}
}