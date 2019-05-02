import Foundation
import StdMsgs
import RosTime


extension smach_msgs {
/// The path to this node in the server
/// The initial state description
/// Effects an arc from the top state to each one
/// The current state description
/// A pickled user data structure
/// i.e. the UserData's internal dictionary
/// Debugging info string
public struct SmachContainerStatus: Message {
public static var md5sum: String = "5ba2bb79ac19e3842d562a191f2a675b"
public static var datatype = "smach_msgs/SmachContainerStatus"
public static var definition = """
Header header

# The path to this node in the server
string path

# The initial state description
# Effects an arc from the top state to each one
string[] initial_states

# The current state description
string[] active_states

# A pickled user data structure
# i.e. the UserData's internal dictionary
string local_data

# Debugging info string
string info
"""
public static var hasHeader = false

public var header: std_msgs.header
public var path: String
public var initial_states: [String]
public var active_states: [String]
public var local_data: String
public var info: String

public init(header: std_msgs.header, path: String, initial_states: [String], active_states: [String], local_data: String, info: String) {
self.header = header
self.path = path
self.initial_states = initial_states
self.active_states = active_states
self.local_data = local_data
self.info = info
}

public init() {
    header = std_msgs.header()
path = String()
initial_states = [String]()
active_states = [String]()
local_data = String()
info = String()
}

}
}
