import Foundation
import StdMsgs
import RosTime


extension tf2_msgs {

public struct TF2Error: Message {
public static var md5sum: String = "bc6848fd6fd750c92e38575618a4917d"
public static var datatype = "tf2_msgs/TF2Error"
public static var definition = """
uint8 NO_ERROR = 0
uint8 LOOKUP_ERROR = 1
uint8 CONNECTIVITY_ERROR = 2
uint8 EXTRAPOLATION_ERROR = 3
uint8 INVALID_ARGUMENT_ERROR = 4
uint8 TIMEOUT_ERROR = 5
uint8 TRANSFORM_ERROR = 6

uint8 error
string error_string
"""
public static var hasHeader = false

public let NO_ERROR: UInt8 = 0
public let LOOKUP_ERROR: UInt8 = 1
public let CONNECTIVITY_ERROR: UInt8 = 2
public let EXTRAPOLATION_ERROR: UInt8 = 3
public let INVALID_ARGUMENT_ERROR: UInt8 = 4
public let TIMEOUT_ERROR: UInt8 = 5
public let TRANSFORM_ERROR: UInt8 = 6
public var error: UInt8
public var error_string: String

public init(error: UInt8, error_string: String) {
self.error = error
self.error_string = error_string
}

public init() {
    error = UInt8()
error_string = String()
}

}
}
