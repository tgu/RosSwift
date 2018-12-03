import Foundation
import StdMsgs
import RosTime


extension sensor_msgs {
/// Single temperature reading.
public struct Temperature: Message {
public static var md5sum: String = "ff71b307acdbe7c871a5a6d7ed359100"
public static var datatype = "sensor_msgs/Temperature"
public static var definition = """
# Single temperature reading.

 Header header           # timestamp is the time the temperature was measured
                         # frame_id is the location of the temperature reading

 float64 temperature     # Measurement of the Temperature in Degrees Celsius

 float64 variance        # 0 is interpreted as variance unknown
"""
public static var hasHeader = false

public var header: std_msgs.header
public var temperature: Float64
public var variance: Float64

public init(header: std_msgs.header, temperature: Float64, variance: Float64) {
self.header = header
self.temperature = temperature
self.variance = variance
}

public init() {
    header = std_msgs.header()
temperature = Float64()
variance = Float64()
}

}
}