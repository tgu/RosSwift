//
//  File.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2019-04-25.
//

import Foundation
import StdMsgs


struct EmptyRequest: ServiceMessage {
    static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
    static let srvMd5sum = Empty.md5sum
    static let srvDatatype = Empty.datatype
    static let datatype = "roscpp/EmptyRequest"
    static let hasHeader = false
    static let definition = "\n"
}

struct EmptyResponse: ServiceMessage {
    static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
    static let srvMd5sum = Empty.md5sum
    static let srvDatatype = Empty.datatype
    static let datatype = "roscpp/SetLoggerLevelResponse"
    static let hasHeader = false
    static let definition = "\n"
}

struct Empty {
    typealias Request = EmptyRequest
    typealias Response = EmptyResponse

    static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
    static let datatype = "roscpp/Empty"
}
