import Foundation
import StdMsgs
import RosTime


extension smach_msgs {
/// The path to the node in the server
/// The desired initial state(s)
/// Initial values for the local user data of the state machine
/// A pickled user data structure
/// i.e. the UserData's internal dictionary
public struct SmachContainerInitialStatusCmd: Message {
public static var md5sum: String = "45f8cf31fc29b829db77f23001f788d6"
public static var datatype = "smach_msgs/SmachContainerInitialStatusCmd"
public static var definition = """
# The path to the node in the server
string path

# The desired initial state(s)
string[] initial_states

# Initial values for the local user data of the state machine
# A pickled user data structure
# i.e. the UserData's internal dictionary
string local_data
"""
public static var hasHeader = false

public var path: String
public var initial_states: [String]
public var local_data: String

public init(path: String, initial_states: [String], local_data: String) {
self.path = path
self.initial_states = initial_states
self.local_data = local_data
}

public init() {
    path = String()
initial_states = [String]()
local_data = String()
}

}
}
