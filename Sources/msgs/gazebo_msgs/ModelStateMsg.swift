import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension gazebo_msgs {
/// Set Gazebo Model pose and twist
public struct ModelState: Message {
public static var md5sum: String = "9330fd35f2fcd82d457e54bd54e10593"
public static var datatype = "gazebo_msgs/ModelState"
public static var definition = """
# Set Gazebo Model pose and twist
string model_name           # model to set state (pose and twist)
geometry_msgs/Pose pose     # desired pose in reference frame
geometry_msgs/Twist twist   # desired twist in reference frame
string reference_frame      # set pose/twist relative to the frame of this entity (Body/Model)
                            # leave empty or "world" or "map" defaults to world-frame
"""
public static var hasHeader = false

public var model_name: String
public var pose: geometry_msgs.Pose
public var twist: geometry_msgs.Twist
public var reference_frame: String

public init(model_name: String, pose: geometry_msgs.Pose, twist: geometry_msgs.Twist, reference_frame: String) {
self.model_name = model_name
self.pose = pose
self.twist = twist
self.reference_frame = reference_frame
}

public init() {
    model_name = String()
pose = geometry_msgs.Pose()
twist = geometry_msgs.Twist()
reference_frame = String()
}

}
}
