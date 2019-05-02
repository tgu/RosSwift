import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension gazebo_msgs {
/// @todo: FIXME: sets pose and twist of a link.  All children link poses/twists of the URDF tree are not updated accordingly, but should be.
public struct LinkState: Message {
public static var md5sum: String = "0818ebbf28ce3a08d48ab1eaa7309ebe"
public static var datatype = "gazebo_msgs/LinkState"
public static var definition = """
# @todo: FIXME: sets pose and twist of a link.  All children link poses/twists of the URDF tree are not updated accordingly, but should be.
string link_name            # link name, link_names are in gazebo scoped name notation, [model_name::body_name]
geometry_msgs/Pose pose     # desired pose in reference frame
geometry_msgs/Twist twist   # desired twist in reference frame
string reference_frame      # set pose/twist relative to the frame of this link/body
                            # leave empty or "world" or "map" defaults to world-frame
"""
public static var hasHeader = false

public var link_name: String
public var pose: geometry_msgs.Pose
public var twist: geometry_msgs.Twist
public var reference_frame: String

public init(link_name: String, pose: geometry_msgs.Pose, twist: geometry_msgs.Twist, reference_frame: String) {
self.link_name = link_name
self.pose = pose
self.twist = twist
self.reference_frame = reference_frame
}

public init() {
    link_name = String()
pose = geometry_msgs.Pose()
twist = geometry_msgs.Twist()
reference_frame = String()
}

}
}
