import Foundation
import RosTime

extension std_msgs {
    public struct time: Message {
        public var data : RosTime.TimeBase
        public static var md5sum: String = "cd7166c74c552c311fbcc2fe5a7bc289"
        public static var datatype = "std_msgs/Time"
        public static var definition = "time data"
        public static var hasHeader = false


        public init(_ value: RosTime.TimeBase) {
            self.data = value
        }

        public init() {
            self.data = RosTime.TimeBase()
        }

    }
}