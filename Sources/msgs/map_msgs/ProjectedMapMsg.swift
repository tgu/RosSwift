import Foundation
import StdMsgs
import RosTime
import nav_msgs

extension map_msgs {

public struct ProjectedMap: Message {
public static var md5sum: String = "7bbe8f96e45089681dc1ea7d023cbfca"
public static var datatype = "map_msgs/ProjectedMap"
public static var definition = """
nav_msgs/OccupancyGrid map
float64 min_z
float64 max_z
"""
public static var hasHeader = false

public var map: nav_msgs.OccupancyGrid
public var min_z: Float64
public var max_z: Float64

public init(map: nav_msgs.OccupancyGrid, min_z: Float64, max_z: Float64) {
self.map = map
self.min_z = min_z
self.max_z = max_z
}

public init() {
    map = nav_msgs.OccupancyGrid()
min_z = Float64()
max_z = Float64()
}

}
}
