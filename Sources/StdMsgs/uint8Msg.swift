import Foundation
import RosTime

extension std_msgs {
    public struct uint8: Message {
        public var data : UInt8
        public static var md5sum: String = "7c8164229e7d2c17eb95e9231617fdee"
        public static var datatype = "std_msgs/UInt8"
        public static var definition = "uint8 data"
        public static var hasHeader = false


        public init(_ value: UInt8) {
            self.data = value
        }

        public init() {
            self.data = UInt8()
        }

    }
}