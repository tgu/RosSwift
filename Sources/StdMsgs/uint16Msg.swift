import Foundation
import RosTime

extension std_msgs {
    public struct uint16: Message {
        public var data : UInt16
        public static var md5sum: String = "1df79edf208b629fe6b81923a544552d"
        public static var datatype = "std_msgs/UInt16"
        public static var definition = "uint16 data"
        public static var hasHeader = false


        public init(_ value: UInt16) {
            self.data = value
        }

        public init() {
            self.data = UInt16()
        }

    }
}