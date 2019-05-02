import Foundation
import StdMsgs
import RosTime


extension controller_manager_msgs {
/// This message contains the state of one realtime controller
/// that was spawned in the controller manager
/// the name of the controller
/// the type of the controller
/// the time at which these controller statistics were measured
/// bool that indicates if the controller is currently
/// in a running or a stopped state
/// the maximum time the update loop of the controller ever needed to complete
/// the average time the update loop of the controller needs to complete.
/// the average is computed in a sliding time window.
/// the variance on the time the update loop of the controller needs to complete.
/// the variance applies to a sliding time window.
/// the number of times this controller broke the realtime loop
/// the timestamp of the last time this controller broke the realtime loop
public struct ControllerStatistics: Message {
public static var md5sum: String = "697780c372c8d8597a1436d0e2ad3ba8"
public static var datatype = "controller_manager_msgs/ControllerStatistics"
public static var definition = """
# This message contains the state of one realtime controller
# that was spawned in the controller manager

# the name of the controller
string name

# the type of the controller
string type

# the time at which these controller statistics were measured
time timestamp

# bool that indicates if the controller is currently
# in a running or a stopped state
bool running

# the maximum time the update loop of the controller ever needed to complete
duration max_time

# the average time the update loop of the controller needs to complete.
# the average is computed in a sliding time window.
duration mean_time

# the variance on the time the update loop of the controller needs to complete.
# the variance applies to a sliding time window.
duration variance_time

# the number of times this controller broke the realtime loop
int32 num_control_loop_overruns

# the timestamp of the last time this controller broke the realtime loop
time time_last_control_loop_overrun
"""
public static var hasHeader = false

public var name: String
public var type: String
public var timestamp: RosTime.TimeBase
public var running: Bool
public var max_time: RosTime.Duration
public var mean_time: RosTime.Duration
public var variance_time: RosTime.Duration
public var num_control_loop_overruns: Int32
public var time_last_control_loop_overrun: RosTime.TimeBase

public init(name: String, type: String, timestamp: RosTime.TimeBase, running: Bool, max_time: RosTime.Duration, mean_time: RosTime.Duration, variance_time: RosTime.Duration, num_control_loop_overruns: Int32, time_last_control_loop_overrun: RosTime.TimeBase) {
self.name = name
self.type = type
self.timestamp = timestamp
self.running = running
self.max_time = max_time
self.mean_time = mean_time
self.variance_time = variance_time
self.num_control_loop_overruns = num_control_loop_overruns
self.time_last_control_loop_overrun = time_last_control_loop_overrun
}

public init() {
    name = String()
type = String()
timestamp = RosTime.TimeBase()
running = Bool()
max_time = RosTime.Duration()
mean_time = RosTime.Duration()
variance_time = RosTime.Duration()
num_control_loop_overruns = Int32()
time_last_control_loop_overrun = RosTime.TimeBase()
}

}
}
