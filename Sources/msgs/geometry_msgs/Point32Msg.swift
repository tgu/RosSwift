import Foundation
import StdMsgs
import RosTime


extension geometry_msgs {
public struct Point32: Message {
public static var md5sum: String = "cc153912f1453b708d221682bc23d9ac"
public static var datatype = "geometry_msgs/Point32"
public static var definition = """
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
"""
public static var hasHeader = false

public var x: Float32
public var y: Float32
public var z: Float32

public init(x: Float32, y: Float32, z: Float32) {
self.x = x
self.y = y
self.z = z
}

public init() {
    x = Float32()
y = Float32()
z = Float32()
}

}
}