import Foundation
import RosTime

extension std_msgs {
    public struct int64: Message {
        public var data: Int64
        public static var md5sum: String = "34add168574510e6e17f5d23ecc077ef"
        public static var datatype = "std_msgs/Int64"
        public static var definition = "int64 data"
        public static var hasHeader = false


        public init(_ value: Int64) {
            self.data = value
        }

    }
}