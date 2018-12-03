import Foundation
import RosTime

extension std_msgs {
    public struct int16: Message {
        public var data: Int16
        public static var md5sum: String = "8524586e34fbd7cb1c08c5f5f1ca0e57"
        public static var datatype = "std_msgs/Int16"
        public static var definition = "int16 data"
        public static var hasHeader = false


        public init(_ value: Int16) {
            self.data = value
        }

    }
}