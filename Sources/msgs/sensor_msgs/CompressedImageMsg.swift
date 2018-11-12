import Foundation
import StdMsgs
import RosTime


extension sensor_msgs {
public struct CompressedImage: Message {
public static var md5sum: String = "8f7a12909da2c9d3332d540a0977563f"
public static var datatype = "sensor_msgs/CompressedImage"
public static var definition = """
# This message contains a compressed image

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image

string format        # Specifies the format of the data
                     #   Acceptable values:
                     #     jpeg, png
uint8[] data         # Compressed image buffer
"""
public static var hasHeader = false

public var header: std_msgs.header
public var format: String
public var data: [UInt8]

public init(header: std_msgs.header, format: String, data: [UInt8]) {
self.header = header
self.format = format
self.data = data
}

public init() {
    header = std_msgs.header()
format = String()
data = [UInt8]()
}

}
}