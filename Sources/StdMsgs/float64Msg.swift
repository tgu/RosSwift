import Foundation
import RosTime

extension std_msgs {
    public struct float64: Message {
        public var data : Float64
        public static var md5sum: String = "fdb28210bfa9d7c91146260178d9a584"
        public static var datatype = "std_msgs/Float64"
        public static var definition = "float64 data"
        public static var hasHeader = false


        public init(_ value: Float64) {
            self.data = value
        }

        public init() {
            self.data = Float64()
        }

    }
}