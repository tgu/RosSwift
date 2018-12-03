import Foundation
import RosTime

extension std_msgs {
    public struct int32: Message {
        public var data: Int32
        public static var md5sum: String = "da5909fbe378aeaf85e547e830cc1bb7"
        public static var datatype = "std_msgs/Int32"
        public static var definition = "int32 data"
        public static var hasHeader = false


        public init(_ value: Int32) {
            self.data = value
        }

    }
}