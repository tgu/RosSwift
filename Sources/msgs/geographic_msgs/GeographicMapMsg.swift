// Generated by msgbuilder 2020-05-12 12:09:05 +0000

import StdMsgs

extension geographic_msgs {
	/// Geographic map for a specified region.
	public struct GeographicMap: MessageWithHeader {
		public static let md5sum: String = "0f4ce6d2ebf9ac9c7c4f3308f6ae0731"
		public static let datatype = "geographic_msgs/GeographicMap"
		public static let definition = """
			# Geographic map for a specified region.
			Header header            # stamp specifies time
			                         # frame_id (normally /map)
			uuid_msgs/UniqueID id    # identifier for this map
			BoundingBox  bounds      # 2D bounding box containing map
			WayPoint[]   points      # way-points
			MapFeature[] features    # map features
			KeyValue[]   props       # map properties
			"""

	
		public var header: std_msgs.Header
		public var id: uuid_msgs.UniqueID
		public var bounds: BoundingBox
		public var points: [WayPoint]
		public var features: [MapFeature]
		public var props: [KeyValue]

		public init(header: std_msgs.Header, id: uuid_msgs.UniqueID, bounds: BoundingBox, points: [WayPoint], features: [MapFeature], props: [KeyValue]) {
			self.header = header
			self.id = id
			self.bounds = bounds
			self.points = points
			self.features = features
			self.props = props
		}

		public init() {
			header = std_msgs.Header()
			id = uuid_msgs.UniqueID()
			bounds = BoundingBox()
			points = [WayPoint]()
			features = [MapFeature]()
			props = [KeyValue]()
		}
	}
}