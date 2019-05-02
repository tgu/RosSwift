import Foundation
import StdMsgs
import RosTime


extension dynamic_reconfigure {

public struct StrParameter: Message {
public static var md5sum: String = "bc6ccc4a57f61779c8eaae61e9f422e0"
public static var datatype = "dynamic_reconfigure/StrParameter"
public static var definition = """
string name
string value
"""
public static var hasHeader = false

public var name: String
public var value: String

public init(name: String, value: String) {
self.name = name
self.value = value
}

public init() {
    name = String()
value = String()
}

}
}
