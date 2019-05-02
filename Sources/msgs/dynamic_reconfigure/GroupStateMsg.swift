import Foundation
import StdMsgs
import RosTime


extension dynamic_reconfigure {

public struct GroupState: Message {
public static var md5sum: String = "a2d87f51dc22930325041a2f8b1571f8"
public static var datatype = "dynamic_reconfigure/GroupState"
public static var definition = """
string name
bool state
int32 id
int32 parent
"""
public static var hasHeader = false

public var name: String
public var state: Bool
public var id: Int32
public var parent: Int32

public init(name: String, state: Bool, id: Int32, parent: Int32) {
self.name = name
self.state = state
self.id = id
self.parent = parent
}

public init() {
    name = String()
state = Bool()
id = Int32()
parent = Int32()
}

}
}
