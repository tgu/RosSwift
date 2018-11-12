import Foundation
import RosTime

extension std_msgs {
    public struct uint64: Message {
        public var data : UInt64
        public static var md5sum: String = "1b2a79973e8bf53d7b53acb71299cb57"
        public static var datatype = "std_msgs/UInt64"
        public static var definition = "uint64 data"
        public static var hasHeader = false


        public init(_ value: UInt64) {
            self.data = value
        }

        public init() {
            self.data = UInt64()
        }

    }
}