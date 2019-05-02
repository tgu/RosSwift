import Foundation
import StdMsgs
import RosTime


extension shape_msgs {
	/// Define box, sphere, cylinder, cone 
	/// All shapes are defined to have their bounding boxes centered around 0,0,0.
	/// The type of the shape
	/// The dimensions of the shape
	/// The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array
	/// For the BOX type, the X, Y, and Z dimensions are the length of the corresponding
	/// sides of the box.
	/// For the SPHERE type, only one component is used, and it gives the radius of
	/// the sphere.
	/// For the CYLINDER and CONE types, the center line is oriented along
	/// the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component
	/// of dimensions gives the height of the cylinder (cone).  The
	/// CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the
	/// radius of the base of the cylinder (cone).  Cone and cylinder
	/// primitives are defined to be circular. The tip of the cone is
	/// pointing up, along +Z axis.
	public struct SolidPrimitive: Message {
		public static let md5sum: String = "6ec4c64b44cadf0aba60f1be82e99575"
		public static let datatype = "shape_msgs/SolidPrimitive"
		public static let definition = """
			# Define box, sphere, cylinder, cone 
			# All shapes are defined to have their bounding boxes centered around 0,0,0.
			uint8 BOX=1
			uint8 SPHERE=2
			uint8 CYLINDER=3
			uint8 CONE=4
			# The type of the shape
			uint8 type
			# The dimensions of the shape
			float64[] dimensions
			# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array
			# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding
			# sides of the box.
			uint8 BOX_X=0
			uint8 BOX_Y=1
			uint8 BOX_Z=2
			# For the SPHERE type, only one component is used, and it gives the radius of
			# the sphere.
			uint8 SPHERE_RADIUS=0
			# For the CYLINDER and CONE types, the center line is oriented along
			# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component
			# of dimensions gives the height of the cylinder (cone).  The
			# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the
			# radius of the base of the cylinder (cone).  Cone and cylinder
			# primitives are defined to be circular. The tip of the cone is
			# pointing up, along +Z axis.
			uint8 CYLINDER_HEIGHT=0
			uint8 CYLINDER_RADIUS=1
			uint8 CONE_HEIGHT=0
			uint8 CONE_RADIUS=1
			"""

		public static let hasHeader = false

		public static let BOX: UInt8 = 1
		public static let SPHERE: UInt8 = 2
		public static let CYLINDER: UInt8 = 3
		public static let CONE: UInt8 = 4
		public static let BOX_X: UInt8 = 0
		public static let BOX_Y: UInt8 = 1
		public static let BOX_Z: UInt8 = 2
		public static let SPHERE_RADIUS: UInt8 = 0
		public static let CYLINDER_HEIGHT: UInt8 = 0
		public static let CYLINDER_RADIUS: UInt8 = 1
		public static let CONE_HEIGHT: UInt8 = 0
		public static let CONE_RADIUS: UInt8 = 1
		public var type: UInt8
		public var dimensions: [Float64]

		public init(type: UInt8, dimensions: [Float64]) {
			self.type = type
			self.dimensions = dimensions
		}

		public init() {
			type = UInt8()
			dimensions = [Float64]()
		}
	}
}