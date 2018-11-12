import Foundation
import StdMsgs
import RosTime


extension sensor_msgs {
public struct NavSatStatus: Message {
public static var md5sum: String = "331cdbddfa4bc96ffc3b9ad98900a54c"
public static var datatype = "sensor_msgs/NavSatStatus"
public static var definition = """
# Navigation Satellite fix status for any Global Navigation Satellite System

# Whether to output an augmented fix is determined by both the fix
# type and the last time differential corrections were received.  A
# fix is valid when status >= STATUS_FIX.

int8 STATUS_NO_FIX =  -1        # unable to fix position
int8 STATUS_FIX =      0        # unaugmented fix
int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

int8 status

# Bits defining which Global Navigation Satellite System signals were
# used by the receiver.

uint16 SERVICE_GPS =     1
uint16 SERVICE_GLONASS = 2
uint16 SERVICE_COMPASS = 4      # includes BeiDou.
uint16 SERVICE_GALILEO = 8

uint16 service
"""
public static var hasHeader = false

public let STATUS_NO_FIX : Int8 = -1
public let STATUS_FIX : Int8 = 0
public let STATUS_SBAS_FIX : Int8 = 1
public let STATUS_GBAS_FIX : Int8 = 2
public var status: Int8
public let SERVICE_GPS : UInt16 = 1
public let SERVICE_GLONASS : UInt16 = 2
public let SERVICE_COMPASS : UInt16 = 4
public let SERVICE_GALILEO : UInt16 = 8
public var service: UInt16

public init(status: Int8, service: UInt16) {
self.status = status
self.service = service
}

public init() {
    status = Int8()
service = UInt16()
}

}
}