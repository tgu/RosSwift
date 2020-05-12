//
//  ServiceMessage.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-24.
//

import Foundation
import StdMsgs


public struct TestStringStringRequest: ServiceMessage {
    public var data: String
    public static var md5sum: String = "992ce8a1687cec8c8bd883ec73ca41d1"
    public static var datatype = "std_msgs/String"
    public static var definition = "string data"
    public static var srvMd5sum: String = TestStringString.md5sum
    public static var srvDatatype: String = TestStringString.datatype

    public init(_ value: String) {
        self.data = value
    }

    public init() {
        self.data = String()
    }

}

public struct TestStringStringResponse: ServiceMessage {
    public var data: String
    public static var md5sum: String = "992ce8a1687cec8c8bd883ec73ca41d1"
    public static var datatype = "std_msgs/String"
    public static var definition = "string data"
    public static var srvMd5sum: String = TestStringString.md5sum
    public static var srvDatatype: String = TestStringString.datatype

    public init(_ value: String) {
        self.data = value
    }

    public init() {
        self.data = String()
    }

}


public struct TestStringString: ServiceProt {
    public typealias Request = TestStringStringRequest
    public typealias Response = TestStringStringResponse
    public static let md5sum = "a9e4701c829f791367680c5f8ed06ff4"
    public static let datatype = "beginners_tutorial/TestStringString"

    public var request = TestStringStringRequest()
    public var response = TestStringStringResponse()
}
