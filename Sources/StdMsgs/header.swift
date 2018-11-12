//
//  header.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-16.
//

import Foundation
import RosTime

extension std_msgs {
    public struct header: Message {
        public static var md5sum: String = "2176decaecbce78abc3b96ef049fabed"
        public static var datatype = "std_msgs/Header"
        public static var definition = """
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.
#
# sequence ID: consecutively increasing ID
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
"""
        public static var hasHeader = false

        public var seq: UInt32
        public var stamp: RosTime.Time
        public var frameID: UInt8

        public init(seq: UInt32, stamp: RosTime.Time, frameID: UInt8 ) {
            self.seq = seq
            self.stamp = stamp
            self.frameID = frameID
        }

        public init() {
            self.seq = 0
            self.stamp = RosTime.Time()
            self.frameID = 0
        }
    }
}
