import Foundation
import StdMsgs
import RosTime


extension bond {

public struct Constants: Message {
public static var md5sum: String = "6fc594dc1d7bd7919077042712f8c8b0"
public static var datatype = "bond/Constants"
public static var definition = """
float32 DEAD_PUBLISH_PERIOD = 0.05
float32 DEFAULT_CONNECT_TIMEOUT = 10.0
float32 DEFAULT_HEARTBEAT_TIMEOUT = 4.0
float32 DEFAULT_DISCONNECT_TIMEOUT = 2.0
float32 DEFAULT_HEARTBEAT_PERIOD = 1.0

string DISABLE_HEARTBEAT_TIMEOUT_PARAM=/bond_disable_heartbeat_timeout
"""
public static var hasHeader = false

public let DEAD_PUBLISH_PERIOD: Float32 = 0.05
public let DEFAULT_CONNECT_TIMEOUT: Float32 = 10.0
public let DEFAULT_HEARTBEAT_TIMEOUT: Float32 = 4.0
public let DEFAULT_DISCONNECT_TIMEOUT: Float32 = 2.0
public let DEFAULT_HEARTBEAT_PERIOD: Float32 = 1.0
public var DISABLE_HEARTBEAT_TIMEOUT_PARAM=/bond_disable_heartbeat_timeout: String

public init(DISABLE_HEARTBEAT_TIMEOUT_PARAM=/bond_disable_heartbeat_timeout: String) {
self.DISABLE_HEARTBEAT_TIMEOUT_PARAM=/bond_disable_heartbeat_timeout = DISABLE_HEARTBEAT_TIMEOUT_PARAM=/bond_disable_heartbeat_timeout
}

public init() {
    DISABLE_HEARTBEAT_TIMEOUT_PARAM=/bond_disable_heartbeat_timeout = String()
}

}
}
