import Foundation
import RosTime

extension std_msgs {
    public struct int8: Message {
        public var data : Int8
        public static var md5sum: String = "27ffa0c9c4b8fb8492252bcad9e5c57b"
        public static var datatype = "std_msgs/Int8"
        public static var definition = "int8 data"
        public static var hasHeader = false


        public init(_ value: Int8) {
            self.data = value
        }

        public init() {
            self.data = Int8()
        }

    }
}