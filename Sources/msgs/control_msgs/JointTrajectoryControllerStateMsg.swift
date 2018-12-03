import Foundation
import StdMsgs
import RosTime
import trajectory_msgs

extension control_msgs {

public struct JointTrajectoryControllerState: Message {
public static var md5sum: String = "10817c60c2486ef6b33e97dcd87f4474"
public static var datatype = "control_msgs/JointTrajectoryControllerState"
public static var definition = """
Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint desired
trajectory_msgs/JointTrajectoryPoint actual
trajectory_msgs/JointTrajectoryPoint error  # Redundant, but useful
"""
public static var hasHeader = false

public var header: std_msgs.header
public var joint_names: [String]
public var desired: trajectory_msgs.JointTrajectoryPoint
public var actual: trajectory_msgs.JointTrajectoryPoint
public var error: trajectory_msgs.JointTrajectoryPoint

public init(header: std_msgs.header, joint_names: [String], desired: trajectory_msgs.JointTrajectoryPoint, actual: trajectory_msgs.JointTrajectoryPoint, error: trajectory_msgs.JointTrajectoryPoint) {
self.header = header
self.joint_names = joint_names
self.desired = desired
self.actual = actual
self.error = error
}

public init() {
    header = std_msgs.header()
joint_names = [String]()
desired = trajectory_msgs.JointTrajectoryPoint()
actual = trajectory_msgs.JointTrajectoryPoint()
error = trajectory_msgs.JointTrajectoryPoint()
}

}
}