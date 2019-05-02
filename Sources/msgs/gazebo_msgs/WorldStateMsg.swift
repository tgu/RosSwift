import Foundation
import StdMsgs
import RosTime
import geometry_msgs

extension gazebo_msgs {
/// This is a message that holds data necessary to reconstruct a snapshot of the world
///
/// = Approach to Message Passing =
/// The state of the world is defined by either
///   1. Inertial Model pose, twist
///      * kinematic data - connectivity graph from Model to each Link
///      * joint angles
///      * joint velocities
///      * Applied forces - Body wrench
///        * relative transform from Body to each collision Geom
/// Or
///   2. Inertial (absolute) Body pose, twist, wrench
///      * relative transform from Body to each collision Geom - constant, so not sent over wire
///      * back compute from canonical body info to get Model pose and twist.
///
/// Chooing (2.) because it matches most physics engines out there
///   and is simpler.
///
/// = Future =
/// Consider impacts on using reduced coordinates / graph (parent/child links) approach
///   constraint and physics solvers.
///
/// = Application =
/// This message is used to do the following:
///   * reconstruct the world and objects for sensor generation
///   * stop / start simulation - need pose, twist, wrench of each body
///   * collision detection - need pose of each collision geometry.  velocity/acceleration if
///
/// = Assumptions =
/// Assuming that each (physics) processor node locally already has
///   * collision information - Trimesh for Geoms, etc
///   * relative transforms from Body to Geom - this is assumed to be fixed, do not send oved wire
///   * inertial information - does not vary in time
///   * visual information - does not vary in time
///
public struct WorldState: Message {
public static var md5sum: String = "de1a9de3ab7ba97ac0e9ec01a4eb481e"
public static var datatype = "gazebo_msgs/WorldState"
public static var definition = """
# This is a message that holds data necessary to reconstruct a snapshot of the world
#
# = Approach to Message Passing =
# The state of the world is defined by either
#   1. Inertial Model pose, twist
#      * kinematic data - connectivity graph from Model to each Link
#      * joint angles
#      * joint velocities
#      * Applied forces - Body wrench
#        * relative transform from Body to each collision Geom
# Or
#   2. Inertial (absolute) Body pose, twist, wrench
#      * relative transform from Body to each collision Geom - constant, so not sent over wire
#      * back compute from canonical body info to get Model pose and twist.
#
# Chooing (2.) because it matches most physics engines out there
#   and is simpler.
#
# = Future =
# Consider impacts on using reduced coordinates / graph (parent/child links) approach
#   constraint and physics solvers.
#
# = Application =
# This message is used to do the following:
#   * reconstruct the world and objects for sensor generation
#   * stop / start simulation - need pose, twist, wrench of each body
#   * collision detection - need pose of each collision geometry.  velocity/acceleration if
#
# = Assumptions =
# Assuming that each (physics) processor node locally already has
#   * collision information - Trimesh for Geoms, etc
#   * relative transforms from Body to Geom - this is assumed to be fixed, do not send oved wire
#   * inertial information - does not vary in time
#   * visual information - does not vary in time
#

Header header

string[] name
geometry_msgs/Pose[] pose
geometry_msgs/Twist[] twist
geometry_msgs/Wrench[] wrench
"""
public static var hasHeader = false

public var header: std_msgs.header
public var name: [String]
public var pose: geometry_msgs.[Pose]
public var twist: geometry_msgs.[Twist]
public var wrench: geometry_msgs.[Wrench]

public init(header: std_msgs.header, name: [String], pose: geometry_msgs.[Pose], twist: geometry_msgs.[Twist], wrench: geometry_msgs.[Wrench]) {
self.header = header
self.name = name
self.pose = pose
self.twist = twist
self.wrench = wrench
}

public init() {
    header = std_msgs.header()
name = [String]()
pose = geometry_msgs.[Pose]()
twist = geometry_msgs.[Twist]()
wrench = geometry_msgs.[Wrench]()
}

}
}
