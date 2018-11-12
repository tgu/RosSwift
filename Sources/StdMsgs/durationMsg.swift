import Foundation
import RosTime

extension std_msgs {
    public struct duration: Message {
        public var data : RosTime.Duration
        public static var md5sum: String = "3e286caf4241d664e55f3ad380e2ae46"
        public static var datatype = "std_msgs/Duration"
        public static var definition = "duration data"
        public static var hasHeader = false


        public init(_ value: RosTime.Duration) {
            self.data = value
        }

        public init() {
            self.data = RosTime.Duration()
        }

    }
}