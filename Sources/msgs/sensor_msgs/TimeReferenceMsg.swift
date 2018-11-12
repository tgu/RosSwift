import Foundation
import StdMsgs
import RosTime


extension sensor_msgs {
public struct TimeReference: Message {
public static var md5sum: String = "fded64a0265108ba86c3d38fb11c0c16"
public static var datatype = "sensor_msgs/TimeReference"
public static var definition = """
# Measurement from an external time source not actively synchronized with the system clock.

Header header    # stamp is system time for which measurement was valid
                 # frame_id is not used 

time   time_ref  # corresponding time from this external source
string source    # (optional) name of time source
"""
public static var hasHeader = false

public var header: std_msgs.header
public var time_ref: RosTime.TimeBase
public var source: String

public init(header: std_msgs.header, time_ref: RosTime.TimeBase, source: String) {
self.header = header
self.time_ref = time_ref
self.source = source
}

public init() {
    header = std_msgs.header()
time_ref = RosTime.TimeBase()
source = String()
}

}
}