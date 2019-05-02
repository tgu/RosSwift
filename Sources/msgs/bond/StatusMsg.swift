import Foundation
import StdMsgs
import RosTime


extension bond {
/// Including the timeouts for the bond makes it easier to debug mis-matches
/// between the two sides.
public struct Status: Message {
public static var md5sum: String = "eacc84bf5d65b6777d4c50f463dfb9c8"
public static var datatype = "bond/Status"
public static var definition = """
Header header
string id  # ID of the bond
string instance_id  # Unique ID for an individual in a bond
bool active

# Including the timeouts for the bond makes it easier to debug mis-matches
# between the two sides.
float32 heartbeat_timeout
float32 heartbeat_period
"""
public static var hasHeader = false

public var header: std_msgs.header
public var id: String
public var instance_id: String
public var active: Bool
public var heartbeat_timeout: Float32
public var heartbeat_period: Float32

public init(header: std_msgs.header, id: String, instance_id: String, active: Bool, heartbeat_timeout: Float32, heartbeat_period: Float32) {
self.header = header
self.id = id
self.instance_id = instance_id
self.active = active
self.heartbeat_timeout = heartbeat_timeout
self.heartbeat_period = heartbeat_period
}

public init() {
    header = std_msgs.header()
id = String()
instance_id = String()
active = Bool()
heartbeat_timeout = Float32()
heartbeat_period = Float32()
}

}
}
