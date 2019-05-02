import Foundation
import StdMsgs
import RosTime


extension dynamic_reconfigure {

public struct Config: Message {
public static var md5sum: String = "958f16a05573709014982821e6822580"
public static var datatype = "dynamic_reconfigure/Config"
public static var definition = """
BoolParameter[] bools
IntParameter[] ints
StrParameter[] strs
DoubleParameter[] doubles
GroupState[] groups
"""
public static var hasHeader = false

public var bools: [BoolParameter]
public var ints: [IntParameter]
public var strs: [StrParameter]
public var doubles: [DoubleParameter]
public var groups: [GroupState]

public init(bools: [BoolParameter], ints: [IntParameter], strs: [StrParameter], doubles: [DoubleParameter], groups: [GroupState]) {
self.bools = bools
self.ints = ints
self.strs = strs
self.doubles = doubles
self.groups = groups
}

public init() {
    bools = [BoolParameter]()
ints = [IntParameter]()
strs = [StrParameter]()
doubles = [DoubleParameter]()
groups = [GroupState]()
}

}
}
