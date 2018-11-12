import Foundation
import StdMsgs
import RosTime


extension sensor_msgs {
public struct Range: Message {
public static var md5sum: String = "c005c34273dc426c67a020a87bc24148"
public static var datatype = "sensor_msgs/Range"
public static var definition = """
# Single range reading from an active ranger that emits energy and reports
# one range reading that is valid along an arc at the distance measured. 
# This message is  not appropriate for laser scanners. See the LaserScan
# message if you are working with a laser scanner.

# This message also can represent a fixed-distance (binary) ranger.  This
# sensor will have min_range===max_range===distance of detection.
# These sensors follow REP 117 and will output -Inf if the object is detected
# and +Inf if the object is outside of the detection range.

Header header           # timestamp in the header is the time the ranger
                        # returned the distance reading

# Radiation type enums
# If you want a value added to this list, send an email to the ros-users list
uint8 ULTRASOUND=0
uint8 INFRARED=1

uint8 radiation_type    # the type of radiation used by the sensor
                        # (sound, IR, etc) [enum]

float32 field_of_view   # the size of the arc that the distance reading is
                        # valid for [rad]
                        # the object causing the range reading may have
                        # been anywhere within -field_of_view/2 and
                        # field_of_view/2 at the measured range. 
                        # 0 angle corresponds to the x-axis of the sensor.

float32 min_range       # minimum range value [m]
float32 max_range       # maximum range value [m]
                        # Fixed distance rangers require min_range==max_range

float32 range           # range data [m]
                        # (Note: values < range_min or > range_max
                        # should be discarded)
                        # Fixed distance rangers only output -Inf or +Inf.
                        # -Inf represents a detection within fixed distance.
                        # (Detection too close to the sensor to quantify)
                        # +Inf represents no detection within the fixed distance.
                        # (Object out of range)
"""
public static var hasHeader = false

public var header: std_msgs.header
public var ULTRASOUND=0: UInt8
public var INFRARED=1: UInt8
public var radiation_type: UInt8
public var field_of_view: Float32
public var min_range: Float32
public var max_range: Float32
public var range: Float32

public init(header: std_msgs.header, ULTRASOUND=0: UInt8, INFRARED=1: UInt8, radiation_type: UInt8, field_of_view: Float32, min_range: Float32, max_range: Float32, range: Float32) {
self.header = header
self.ULTRASOUND=0 = ULTRASOUND=0
self.INFRARED=1 = INFRARED=1
self.radiation_type = radiation_type
self.field_of_view = field_of_view
self.min_range = min_range
self.max_range = max_range
self.range = range
}

public init() {
    header = std_msgs.header()
ULTRASOUND=0 = UInt8()
INFRARED=1 = UInt8()
radiation_type = UInt8()
field_of_view = Float32()
min_range = Float32()
max_range = Float32()
range = Float32()
}

}
}