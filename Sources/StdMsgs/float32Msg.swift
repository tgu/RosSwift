import Foundation
import RosTime

extension std_msgs {
    public struct float32: Message {
        public var data : Float32
        public static var md5sum: String = "73fcbf46b49191e672908e50842a83d4"
        public static var datatype = "std_msgs/Float32"
        public static var definition = "float32 data"
        public static var hasHeader = false


        public init(_ value: Float32) {
            self.data = value
        }

        public init() {
            self.data = Float32()
        }

    }
}