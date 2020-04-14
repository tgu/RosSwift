import Foundation
import StdMsgs
import RosTime


extension rospy_tutorials {

public struct HeaderString: Message {
public static var md5sum: String = "c99a9440709e4d4a9716d55b8270d5e7"
public static var datatype = "rospy_tutorials/HeaderString"
public static var definition = """
Header header
string data
"""
public static var hasHeader = false

public var header: std_msgs.Header
public var data: String

public init(header: std_msgs.Header, data: String) {
self.header = header
self.data = data
}

public init() {
    header = std_msgs.Header()
data = String()
}

}
}
