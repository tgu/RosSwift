import Foundation
import StdMsgs
import RosTime


extension sensor_msgs {
/// Single pressure reading.  This message is appropriate for measuring the
public struct FluidPressure: Message {
public static var md5sum: String = "804dc5cea1c5306d6a2eb80b9833befe"
public static var datatype = "sensor_msgs/FluidPressure"
public static var definition = """
# Single pressure reading.  This message is appropriate for measuring the
 # pressure inside of a fluid (air, water, etc).  This also includes
 # atmospheric or barometric pressure.

 # This message is not appropriate for force/pressure contact sensors.

 Header header           # timestamp of the measurement
                         # frame_id is the location of the pressure sensor

 float64 fluid_pressure  # Absolute pressure reading in Pascals.

 float64 variance        # 0 is interpreted as variance unknown
"""
public static var hasHeader = false

public var header: std_msgs.header
public var fluid_pressure: Float64
public var variance: Float64

public init(header: std_msgs.header, fluid_pressure: Float64, variance: Float64) {
self.header = header
self.fluid_pressure = fluid_pressure
self.variance = variance
}

public init() {
    header = std_msgs.header()
fluid_pressure = Float64()
variance = Float64()
}

}
}