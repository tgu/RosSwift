import Foundation
import RosTime

extension std_msgs {
    public struct byte: Message {
        public var data : Int8
        public static var md5sum: String = "ad736a2e8818154c487bb80fe42ce43b"
        public static var datatype = "std_msgs/Byte"
        public static var definition = "byte data"
        public static var hasHeader = false


        public init(_ value: Int8) {
            self.data = value
        }

        public init() {
            self.data = Int8()
        }

    }
}