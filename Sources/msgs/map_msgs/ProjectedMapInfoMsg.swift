import Foundation
import StdMsgs
import RosTime


extension map_msgs {

public struct ProjectedMapInfo: Message {
public static var md5sum: String = "2dc10595ae94de23f22f8a6d2a0eef7a"
public static var datatype = "map_msgs/ProjectedMapInfo"
public static var definition = """
string frame_id
float64 x
float64 y
float64 width
float64 height
float64 min_z
float64 max_z
"""
public static var hasHeader = false

public var frame_id: String
public var x: Float64
public var y: Float64
public var width: Float64
public var height: Float64
public var min_z: Float64
public var max_z: Float64

public init(frame_id: String, x: Float64, y: Float64, width: Float64, height: Float64, min_z: Float64, max_z: Float64) {
self.frame_id = frame_id
self.x = x
self.y = y
self.width = width
self.height = height
self.min_z = min_z
self.max_z = max_z
}

public init() {
    frame_id = String()
x = Float64()
y = Float64()
width = Float64()
height = Float64()
min_z = Float64()
max_z = Float64()
}

}
}
