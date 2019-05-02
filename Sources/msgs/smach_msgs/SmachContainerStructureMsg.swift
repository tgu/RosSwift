import Foundation
import StdMsgs
import RosTime


extension smach_msgs {
/// The path to this node in the server
/// The children of this node
/// The outcome edges
/// Each index across these arrays denote one edge
/// The potential outcomes from this container
public struct SmachContainerStructure: Message {
public static var md5sum: String = "3d3d1e0d0f99779ee9e58101a5dcf7ea"
public static var datatype = "smach_msgs/SmachContainerStructure"
public static var definition = """
Header header

# The path to this node in the server
string path

# The children of this node
string[] children

# The outcome edges
# Each index across these arrays denote one edge
string[] internal_outcomes
string[] outcomes_from
string[] outcomes_to

# The potential outcomes from this container
string[] container_outcomes
"""
public static var hasHeader = false

public var header: std_msgs.header
public var path: String
public var children: [String]
public var internal_outcomes: [String]
public var outcomes_from: [String]
public var outcomes_to: [String]
public var container_outcomes: [String]

public init(header: std_msgs.header, path: String, children: [String], internal_outcomes: [String], outcomes_from: [String], outcomes_to: [String], container_outcomes: [String]) {
self.header = header
self.path = path
self.children = children
self.internal_outcomes = internal_outcomes
self.outcomes_from = outcomes_from
self.outcomes_to = outcomes_to
self.container_outcomes = container_outcomes
}

public init() {
    header = std_msgs.header()
path = String()
children = [String]()
internal_outcomes = [String]()
outcomes_from = [String]()
outcomes_to = [String]()
container_outcomes = [String]()
}

}
}
