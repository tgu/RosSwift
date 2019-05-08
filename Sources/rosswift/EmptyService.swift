//
//  File.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2019-04-25.
//

import Foundation
import StdMsgs


public struct EmptyRequest: ServiceMessage {
    public static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
    public static let srvMd5sum = EmptySrv.md5sum
    public static let srvDatatype = EmptySrv.datatype
    public static let datatype = "roscpp/EmptyRequest"
    public static let hasHeader = false
    public static let definition = "\n"

    public init() {}
}

public struct EmptyResponse: ServiceMessage {
    public static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
    public static let srvMd5sum = EmptySrv.md5sum
    public static let srvDatatype = EmptySrv.datatype
    public static let datatype = "roscpp/SetLoggerLevelResponse"
    public static let hasHeader = false
    public static let definition = "\n"

    public init() {}
}

public enum EmptySrv {
    public typealias Request = EmptyRequest
    public typealias Response = EmptyResponse

    public static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
    public static let datatype = "roscpp/Empty"
}
