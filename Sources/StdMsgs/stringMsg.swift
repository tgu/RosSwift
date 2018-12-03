import Foundation
import RosTime

extension std_msgs {
    public struct string: Message {
        public var data: String
        public static var md5sum: String = "992ce8a1687cec8c8bd883ec73ca41d1"
        public static var datatype = "std_msgs/String"
        public static var definition = "string data"
        public static var hasHeader = false


        public init(_ value: String) {
            self.data = value
        }

    }
}