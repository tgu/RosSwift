import Foundation
import StdMsgs
import RosTime


extension rospy_tutorials {

public struct Floats: Message {
public static var md5sum: String = "420cd38b6b071cd49f2970c3e2cee511"
public static var datatype = "rospy_tutorials/Floats"
public static var definition = """
float32[] data
"""
public static var hasHeader = false

public var data: [Float32]

public init(data: [Float32]) {
self.data = data
}

public init() {
    data = [Float32]()
}

}
}
