import Foundation
import StdMsgs
import RosTime
import controller_manager_msgs

extension controller_manager_msgs {

public struct ControllerState: Message {
public static var md5sum: String = "aeb6b261d97793ab74099a3740245272"
public static var datatype = "controller_manager_msgs/ControllerState"
public static var definition = """
string name
string state
string type
controller_manager_msgs/HardwareInterfaceResources[] claimed_resources
"""
public static var hasHeader = false

public var name: String
public var state: String
public var type: String
public var claimed_resources: controller_manager_msgs.[HardwareInterfaceResources]

public init(name: String, state: String, type: String, claimed_resources: controller_manager_msgs.[HardwareInterfaceResources]) {
self.name = name
self.state = state
self.type = type
self.claimed_resources = claimed_resources
}

public init() {
    name = String()
state = String()
type = String()
claimed_resources = controller_manager_msgs.[HardwareInterfaceResources]()
}

}
}
