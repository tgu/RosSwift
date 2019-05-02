import Foundation
import StdMsgs
import RosTime
import std_msgs
import controller_manager_msgs

extension controller_manager_msgs {

public struct ControllersStatistics: Message {
public static var md5sum: String = "a154c347736773e3700d1719105df29d"
public static var datatype = "controller_manager_msgs/ControllersStatistics"
public static var definition = """
std_msgs/Header header
controller_manager_msgs/ControllerStatistics[] controller
"""
public static var hasHeader = false

public var header: std_msgs.std_msgs.header
public var controller: controller_manager_msgs.[ControllerStatistics]

public init(header: std_msgs.std_msgs.header, controller: controller_manager_msgs.[ControllerStatistics]) {
self.header = header
self.controller = controller
}

public init() {
    header = std_msgs.std_msgs.header()
controller = controller_manager_msgs.[ControllerStatistics]()
}

}
}
