import Foundation
import StdMsgs
import RosTime


extension dynamic_reconfigure {

public struct Group: Message {
public static var md5sum: String = "9e8cd9e9423c94823db3614dd8b1cf7a"
public static var datatype = "dynamic_reconfigure/Group"
public static var definition = """
string name
string type
ParamDescription[] parameters
int32 parent 
int32 id
"""
public static var hasHeader = false

public var name: String
public var type: String
public var parameters: [ParamDescription]
public var parent: Int32
public var id: Int32

public init(name: String, type: String, parameters: [ParamDescription], parent: Int32, id: Int32) {
self.name = name
self.type = type
self.parameters = parameters
self.parent = parent
self.id = id
}

public init() {
    name = String()
type = String()
parameters = [ParamDescription]()
parent = Int32()
id = Int32()
}

}
}
