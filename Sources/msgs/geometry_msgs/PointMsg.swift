import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
/// This contains the position of a point in free space
public struct Point: Message {
public static var md5sum: String = "4a842b65f413084dc2b10fb484ea7f17"
public static var datatype = "geometry_msgs/Point"
public static var definition = """
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
public static var hasHeader = false

public var x: Float64
public var y: Float64
public var z: Float64

public init(x: Float64, y: Float64, z: Float64) {
self.x = x
self.y = y
self.z = z
}

public init() {
    x = Float64()
y = Float64()
z = Float64()
}

}
}