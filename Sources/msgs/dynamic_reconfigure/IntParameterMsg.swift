import Foundation
import StdMsgs
import RosTime


extension dynamic_reconfigure {

public struct IntParameter: Message {
public static var md5sum: String = "65fedc7a0cbfb8db035e46194a350bf1"
public static var datatype = "dynamic_reconfigure/IntParameter"
public static var definition = """
string name
int32 value
"""
public static var hasHeader = false

public var name: String
public var value: Int32

public init(name: String, value: Int32) {
self.name = name
self.value = value
}

public init() {
    name = String()
value = Int32()
}

}
}
