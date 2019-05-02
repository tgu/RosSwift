import Foundation
import StdMsgs
import RosTime


extension dynamic_reconfigure {

public struct DoubleParameter: Message {
public static var md5sum: String = "d8512f27253c0f65f928a67c329cd658"
public static var datatype = "dynamic_reconfigure/DoubleParameter"
public static var definition = """
string name
float64 value
"""
public static var hasHeader = false

public var name: String
public var value: Float64

public init(name: String, value: Float64) {
self.name = name
self.value = value
}

public init() {
    name = String()
value = Float64()
}

}
}
