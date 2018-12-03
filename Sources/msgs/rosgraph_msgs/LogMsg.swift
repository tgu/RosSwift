import Foundation
import StdMsgs
import RosTime


extension rosgraph_msgs {
//////
////// Severity level constants
//////
//////
////// Fields
//////
public struct Log: Message {
public static var md5sum: String = "acffd30cd6b6de30f120938c17c593fb"
public static var datatype = "rosgraph_msgs/Log"
public static var definition = """
##
## Severity level constants
##
byte DEBUG=1 #debug level
byte INFO=2  #general level
byte WARN=4  #warning level
byte ERROR=8 #error level
byte FATAL=16 #fatal/critical level
##
## Fields
##
Header header
byte level
string name # name of the node
string msg # message 
string file # file the message came from
string function # function the message came from
uint32 line # line the message came from
string[] topics # topic names that the node publishes
"""
public static var hasHeader = false

public var DEBUG=1: Int8
public var INFO=2: Int8
public var WARN=4: Int8
public var ERROR=8: Int8
public var FATAL=16: Int8
public var header: std_msgs.header
public var level: Int8
public var name: String
public var msg: String
public var file: String
public var function: String
public var line: UInt32
public var topics: [String]

public init(DEBUG=1: Int8, INFO=2: Int8, WARN=4: Int8, ERROR=8: Int8, FATAL=16: Int8, header: std_msgs.header, level: Int8, name: String, msg: String, file: String, function: String, line: UInt32, topics: [String]) {
self.DEBUG=1 = DEBUG=1
self.INFO=2 = INFO=2
self.WARN=4 = WARN=4
self.ERROR=8 = ERROR=8
self.FATAL=16 = FATAL=16
self.header = header
self.level = level
self.name = name
self.msg = msg
self.file = file
self.function = function
self.line = line
self.topics = topics
}

public init() {
    DEBUG=1 = Int8()
INFO=2 = Int8()
WARN=4 = Int8()
ERROR=8 = Int8()
FATAL=16 = Int8()
header = std_msgs.header()
level = Int8()
name = String()
msg = String()
file = String()
function = String()
line = UInt32()
topics = [String]()
}

}
}