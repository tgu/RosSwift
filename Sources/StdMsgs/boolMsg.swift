import Foundation
import RosTime

extension std_msgs {
    public struct bool: Message {
        public var data: Bool
        public static var md5sum: String = "8b94c1b53db61fb6aed406028ad6332a"
        public static var datatype = "std_msgs/Bool"
        public static var definition = "bool data"
        public static var hasHeader = false


        public init(_ value: Bool) {
            self.data = value
        }

    }
}