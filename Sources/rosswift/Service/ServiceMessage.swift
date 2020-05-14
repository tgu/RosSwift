//
//  ServiceMessage.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-24.
//

import StdMsgs



public struct TestStringString: ServiceProt {
    public static let md5sum = "a9e4701c829f791367680c5f8ed06ff4"
    public static let datatype = "beginners_tutorial/TestStringString"

    public var request: Request
    public var response: Response

    public struct Request: ServiceRequestMessage {
        public typealias ServiceType = TestStringString
        public var data: String
        public static var md5sum: String = "992ce8a1687cec8c8bd883ec73ca41d1"
        public static var datatype = "std_msgs/String"
        public static var definition = "string data"

        public init(_ value: String) {
            self.data = value
        }

        public init() {
            self.data = String()
        }

    }

    public struct Response: ServiceResponseMessage {
        public typealias ServiceType = TestStringString

        public var data: String
        public static var md5sum: String = "992ce8a1687cec8c8bd883ec73ca41d1"
        public static var datatype = "std_msgs/String"
        public static var definition = "string data"

        public init(_ value: String) {
            self.data = value
        }

        public init() {
            self.data = String()
        }

    }


}
