import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension gazebo_msgs {

public struct ContactState: Message {
public static var md5sum: String = "48c0ffb054b8c444f870cecea1ee50d9"
public static var datatype = "gazebo_msgs/ContactState"
public static var definition = """
string info                                   # text info on this contact
string collision1_name                        # name of contact collision1
string collision2_name                        # name of contact collision2
geometry_msgs/Wrench[] wrenches               # list of forces/torques
geometry_msgs/Wrench total_wrench             # sum of forces/torques in every DOF
geometry_msgs/Vector3[] contact_positions     # list of contact position
geometry_msgs/Vector3[] contact_normals       # list of contact normals
float64[] depths                              # list of penetration depths
"""
public static var hasHeader = false

public var info: String
public var collision1_name: String
public var collision2_name: String
public var wrenches: geometry_msgs.[Wrench]
public var total_wrench: geometry_msgs.Wrench
public var contact_positions: geometry_msgs.[Vector3]
public var contact_normals: geometry_msgs.[Vector3]
public var depths: [Float64]

public init(info: String, collision1_name: String, collision2_name: String, wrenches: geometry_msgs.[Wrench], total_wrench: geometry_msgs.Wrench, contact_positions: geometry_msgs.[Vector3], contact_normals: geometry_msgs.[Vector3], depths: [Float64]) {
self.info = info
self.collision1_name = collision1_name
self.collision2_name = collision2_name
self.wrenches = wrenches
self.total_wrench = total_wrench
self.contact_positions = contact_positions
self.contact_normals = contact_normals
self.depths = depths
}

public init() {
    info = String()
collision1_name = String()
collision2_name = String()
wrenches = geometry_msgs.[Wrench]()
total_wrench = geometry_msgs.Wrench()
contact_positions = geometry_msgs.[Vector3]()
contact_normals = geometry_msgs.[Vector3]()
depths = [Float64]()
}

}
}
