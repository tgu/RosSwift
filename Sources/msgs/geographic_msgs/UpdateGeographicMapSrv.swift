// Generated by msgbuilder 2020-05-14 06:57:32 +0000

import StdMsgs

extension geographic_msgs {
	public enum UpdateGeographicMap: ServiceProt {
		public static let md5sum: String = "93db3f1ec099e9f1b7e442d7d397e244"
		public static let datatype = "geographic_msgs/UpdateGeographicMap"

		/// This service updates a geographic map.
		/// Changes to geographic map.
		public struct Request: ServiceRequestMessage {
			public static let md5sum: String = "8d8da723a1fadc5f7621a18b4e72fc3b"
			public static let datatype = "geographic_msgs/UpdateGeographicMapRequest"
			public typealias ServiceType = UpdateGeographicMap
			public static let definition = """
				# This service updates a geographic map.
				# Changes to geographic map.
				GeographicMapChanges updates
				"""
	
		
			public var updates: GeographicMapChanges
	
			public init(updates: GeographicMapChanges) {
				self.updates = updates
			}
	
			public init() {
				updates = GeographicMapChanges()
			}
		}

	
		public struct Response: ServiceResponseMessage {
			public static let md5sum: String = "38b8954d32a849f31d78416b12bff5d1"
			public static let datatype = "geographic_msgs/UpdateGeographicMapResponse"
			public typealias ServiceType = UpdateGeographicMap
			public static let definition = """
				bool   success        # true if the call succeeded
				string status         # more details
				"""
	
		
			public var success: Bool
			public var status: String
	
			public init(success: Bool, status: String) {
				self.success = success
				self.status = status
			}
	
			public init() {
				success = Bool()
				status = String()
			}
		}

	}
}