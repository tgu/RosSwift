import Foundation
import StdMsgs
import RosTime


extension diagnostic_msgs {
public struct KeyValue: Message {
public static var md5sum: String = "cf57fdc6617a881a88c16e768132149c"
public static var datatype = "diagnostic_msgs/KeyValue"
public static var definition = """
string key # what to label this value when viewing
string value # a value to track over time
"""
public static var hasHeader = false

public var key: String
public var value: String

public init(key: String, value: String) {
self.key = key
self.value = value
}

public init() {
    key = String()
value = String()
}

}
}