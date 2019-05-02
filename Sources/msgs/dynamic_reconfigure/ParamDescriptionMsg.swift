import Foundation
import StdMsgs
import RosTime


extension dynamic_reconfigure {

public struct ParamDescription: Message {
public static var md5sum: String = "7434fcb9348c13054e0c3b267c8cb34d"
public static var datatype = "dynamic_reconfigure/ParamDescription"
public static var definition = """
string name
string type
uint32 level
string description
string edit_method
"""
public static var hasHeader = false

public var name: String
public var type: String
public var level: UInt32
public var description: String
public var edit_method: String

public init(name: String, type: String, level: UInt32, description: String, edit_method: String) {
self.name = name
self.type = type
self.level = level
self.description = description
self.edit_method = edit_method
}

public init() {
    name = String()
type = String()
level = UInt32()
description = String()
edit_method = String()
}

}
}
