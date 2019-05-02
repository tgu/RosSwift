import Foundation
import StdMsgs
import RosTime


extension dynamic_reconfigure {
/// This message is deprecated, please use driver_base/SensorLevels instead.
public struct SensorLevels: Message {
public static var md5sum: String = "6322637bee96d5489db6e2127c47602c"
public static var datatype = "dynamic_reconfigure/SensorLevels"
public static var definition = """
# This message is deprecated, please use driver_base/SensorLevels instead.

byte RECONFIGURE_CLOSE = 3  # Parameters that need a sensor to be stopped completely when changed
byte RECONFIGURE_STOP = 1  # Parameters that need a sensor to stop streaming when changed
byte RECONFIGURE_RUNNING = 0 # Parameters that can be changed while a sensor is streaming
"""
public static var hasHeader = false

public let RECONFIGURE_CLOSE: Int8 = 3
public let RECONFIGURE_STOP: Int8 = 1
public let RECONFIGURE_RUNNING: Int8 = 0



public init() {
    
}

}
}
