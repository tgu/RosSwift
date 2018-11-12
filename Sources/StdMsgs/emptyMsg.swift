import Foundation
import RosTime

extension std_msgs {
    public struct empty: Message {
        public var data : Empty
        public static var md5sum: String = "d41d8cd98f00b204e9800998ecf8427e"
        public static var datatype = "std_msgs/Empty"
        public static var definition = ""
        public static var hasHeader = false


        public init(_ value: Empty) {
            self.data = value
        }

        public init() {
            self.data = Empty()
        }

    }
}