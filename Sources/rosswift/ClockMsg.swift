//
//  ClockMsg.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2019-04-25.
//

import Foundation
import RosTime
import StdMsgs

struct RosgraphMsgs {
    struct Clock: Message {
        static let md5sum = "a9c97c1d230cfc112e270351a944ee47"
        static let datatype = "rosgraph_msgs/Clock"
        static let definition = """
                # roslib/Clock is used for publishing simulated time in ROS. \n
                # This message simply communicates the current time.\n
                # For more information, see http://www.ros.org/wiki/Clock\n
                time clock\n
                """
        static let hasHeader = false

        var time: Time
    }
}
