import Foundation
import StdMsgs
import RosTime
import sensor_msgs

extension stereo_msgs {
	/// Separate header for compatibility with current TimeSynchronizer.
	/// Likely to be removed in a later release, use image.header instead.
	/// Floating point disparity image. The disparities are pre-adjusted for any
	/// x-offset between the principal points of the two cameras (in the case
	/// that they are verged). That is: d = x_l - x_r - (cx_l - cx_r)
	/// Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.
	/// Subwindow of (potentially) valid disparity values.
	/// The range of disparities searched.
	/// In the disparity image, any disparity less than min_disparity is invalid.
	/// The disparity search range defines the horopter, or 3D volume that the
	/// stereo algorithm can "see". Points with Z outside of:
	///     Z_min = fT / max_disparity
	///     Z_max = fT / min_disparity
	/// could not be found.
	/// Smallest allowed disparity increment. The smallest achievable depth range
	/// resolution is delta_Z = (Z^2/fT)*delta_d.
	public struct DisparityImage: Message {
		public static let md5sum: String = "04a177815f75271039fa21f16acad8c9"
		public static let datatype = "stereo_msgs/DisparityImage"
		public static let definition = """
			# Separate header for compatibility with current TimeSynchronizer.
			# Likely to be removed in a later release, use image.header instead.
			Header header
			# Floating point disparity image. The disparities are pre-adjusted for any
			# x-offset between the principal points of the two cameras (in the case
			# that they are verged). That is: d = x_l - x_r - (cx_l - cx_r)
			sensor_msgs/Image image
			# Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.
			float32 f # Focal length, pixels
			float32 T # Baseline, world units
			# Subwindow of (potentially) valid disparity values.
			sensor_msgs/RegionOfInterest valid_window
			# The range of disparities searched.
			# In the disparity image, any disparity less than min_disparity is invalid.
			# The disparity search range defines the horopter, or 3D volume that the
			# stereo algorithm can "see". Points with Z outside of:
			#     Z_min = fT / max_disparity
			#     Z_max = fT / min_disparity
			# could not be found.
			float32 min_disparity
			float32 max_disparity
			# Smallest allowed disparity increment. The smallest achievable depth range
			# resolution is delta_Z = (Z^2/fT)*delta_d.
			float32 delta_d
			"""

		public static let hasHeader = true

	
		public var header: std_msgs.Header
		public var image: sensor_msgs.Image
		public var f: Float32
		public var T: Float32
		public var valid_window: sensor_msgs.RegionOfInterest
		public var min_disparity: Float32
		public var max_disparity: Float32
		public var delta_d: Float32

		public init(header: std_msgs.Header, image: sensor_msgs.Image, f: Float32, T: Float32, valid_window: sensor_msgs.RegionOfInterest, min_disparity: Float32, max_disparity: Float32, delta_d: Float32) {
			self.header = header
			self.image = image
			self.f = f
			self.T = T
			self.valid_window = valid_window
			self.min_disparity = min_disparity
			self.max_disparity = max_disparity
			self.delta_d = delta_d
		}

		public init() {
			header = std_msgs.Header()
			image = sensor_msgs.Image()
			f = Float32()
			T = Float32()
			valid_window = sensor_msgs.RegionOfInterest()
			min_disparity = Float32()
			max_disparity = Float32()
			delta_d = Float32()
		}
	}
}