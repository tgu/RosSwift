import Foundation
import StdMsgs
import RosTime


extension controller_manager_msgs {
/// Type of hardware interface
/// List of resources belonging to the hardware interface
public struct HardwareInterfaceResources: Message {
public static var md5sum: String = "f25b55cbf1d1f76e82e5ec9e83f76258"
public static var datatype = "controller_manager_msgs/HardwareInterfaceResources"
public static var definition = """
# Type of hardware interface
string hardware_interface
# List of resources belonging to the hardware interface
string[] resources
"""
public static var hasHeader = false

public var hardware_interface: String
public var resources: [String]

public init(hardware_interface: String, resources: [String]) {
self.hardware_interface = hardware_interface
self.resources = resources
}

public init() {
    hardware_interface = String()
resources = [String]()
}

}
}
