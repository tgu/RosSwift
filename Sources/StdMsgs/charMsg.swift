import Foundation
import RosTime

extension std_msgs {
    public struct char: Message {
        public var data : UInt8
        public static var md5sum: String = "1bf77f25acecdedba0e224b162199717"
        public static var datatype = "std_msgs/Char"
        public static var definition = "char data"
        public static var hasHeader = false


        public init(_ value: UInt8) {
            self.data = value
        }

        public init() {
            self.data = UInt8()
        }

    }
}