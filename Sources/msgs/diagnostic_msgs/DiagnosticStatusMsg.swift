import Foundation
import StdMsgs
import RosTime


extension diagnostic_msgs {
/// This message holds the status of an individual component of the robot.
/// 
/// Possible levels of operations
public struct DiagnosticStatus: Message {
public static var md5sum: String = "d0ce08bc6e5ba34c7754f563a9cabaf1"
public static var datatype = "diagnostic_msgs/DiagnosticStatus"
public static var definition = """
# This message holds the status of an individual component of the robot.
# 

# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

byte level # level of operation enumerated above 
string name # a description of the test/component reporting
string message # a description of the status
string hardware_id # a hardware unique string
KeyValue[] values # an array of values associated with the status
"""
public static var hasHeader = false

public var OK=0: Int8
public var WARN=1: Int8
public var ERROR=2: Int8
public var STALE=3: Int8
public var level: Int8
public var name: String
public var message: String
public var hardware_id: String
public var values: [KeyValue]

public init(OK=0: Int8, WARN=1: Int8, ERROR=2: Int8, STALE=3: Int8, level: Int8, name: String, message: String, hardware_id: String, values: [KeyValue]) {
self.OK=0 = OK=0
self.WARN=1 = WARN=1
self.ERROR=2 = ERROR=2
self.STALE=3 = STALE=3
self.level = level
self.name = name
self.message = message
self.hardware_id = hardware_id
self.values = values
}

public init() {
    OK=0 = Int8()
WARN=1 = Int8()
ERROR=2 = Int8()
STALE=3 = Int8()
level = Int8()
name = String()
message = String()
hardware_id = String()
values = [KeyValue]()
}

}
}