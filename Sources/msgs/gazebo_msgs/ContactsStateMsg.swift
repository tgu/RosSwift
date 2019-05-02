import Foundation
import StdMsgs
import RosTime
import gazebo_msgs

extension gazebo_msgs {

public struct ContactsState: Message {
public static var md5sum: String = "acbcb1601a8e525bf72509f18e6f668d"
public static var datatype = "gazebo_msgs/ContactsState"
public static var definition = """
Header header                                   # stamp
gazebo_msgs/ContactState[] states            # array of geom pairs in contact
"""
public static var hasHeader = false

public var header: std_msgs.header
public var states: gazebo_msgs.[ContactState]

public init(header: std_msgs.header, states: gazebo_msgs.[ContactState]) {
self.header = header
self.states = states
}

public init() {
    header = std_msgs.header()
states = gazebo_msgs.[ContactState]()
}

}
}
