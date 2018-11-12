import Foundation
import RosTime

extension std_msgs {
    public struct uint32: Message {
        public var data : UInt32
        public static var md5sum: String = "304a39449588c7f8ce2df6e8001c5fce"
        public static var datatype = "std_msgs/UInt32"
        public static var definition = "uint32 data"
        public static var hasHeader = false


        public init(_ value: UInt32) {
            self.data = value
        }

        public init() {
            self.data = UInt32()
        }

    }
}