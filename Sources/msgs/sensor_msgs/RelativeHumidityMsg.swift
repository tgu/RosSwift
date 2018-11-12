import Foundation
import StdMsgs
import RosTime


extension sensor_msgs {
public struct RelativeHumidity: Message {
public static var md5sum: String = "8730015b05955b7e992ce29a2678d90f"
public static var datatype = "sensor_msgs/RelativeHumidity"
public static var definition = """
# Single reading from a relative humidity sensor.  Defines the ratio of partial
 # pressure of water vapor to the saturated vapor pressure at a temperature.

 Header header             # timestamp of the measurement
                           # frame_id is the location of the humidity sensor

 float64 relative_humidity # Expression of the relative humidity
                           # from 0.0 to 1.0.
                           # 0.0 is no partial pressure of water vapor
                           # 1.0 represents partial pressure of saturation

 float64 variance          # 0 is interpreted as variance unknown
"""
public static var hasHeader = false

public var header: std_msgs.header
public var relative_humidity: Float64
public var variance: Float64

public init(header: std_msgs.header, relative_humidity: Float64, variance: Float64) {
self.header = header
self.relative_humidity = relative_humidity
self.variance = variance
}

public init() {
    header = std_msgs.header()
relative_humidity = Float64()
variance = Float64()
}

}
}