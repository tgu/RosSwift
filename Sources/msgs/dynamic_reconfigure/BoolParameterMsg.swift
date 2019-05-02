import Foundation
import StdMsgs
import RosTime


extension dynamic_reconfigure {

public struct BoolParameter: Message {
public static var md5sum: String = "23f05028c1a699fb83e22401228c3a9e"
public static var datatype = "dynamic_reconfigure/BoolParameter"
public static var definition = """
string name
bool value
"""
public static var hasHeader = false

public var name: String
public var value: Bool

public init(name: String, value: Bool) {
self.name = name
self.value = value
}

public init() {
    name = String()
value = Bool()
}

}
}
