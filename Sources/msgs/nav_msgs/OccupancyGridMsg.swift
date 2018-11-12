import Foundation
import StdMsgs
import RosTime


extension nav_msgs {
public struct OccupancyGrid: Message {
public static var md5sum: String = "3381f2d731d4076ec5c71b0759edbe4e"
public static var datatype = "nav_msgs/OccupancyGrid"
public static var definition = """
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
"""
public static var hasHeader = false

public var header: std_msgs.header
public var info: MapMetaData
public var data: [Int8]

public init(header: std_msgs.header, info: MapMetaData, data: [Int8]) {
self.header = header
self.info = info
self.data = data
}

public init() {
    header = std_msgs.header()
info = MapMetaData()
data = [Int8]()
}

}
}