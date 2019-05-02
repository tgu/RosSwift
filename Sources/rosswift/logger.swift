//
//  logger.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-11-19.
//

import Foundation
import StdMsgs
import LoggerAPI


struct Logger: Message {

    static let md5sum = "a6069a2ff40db7bd32143dd66e1f408e"
    static let datatype = "roscpp/Logger"
    static let hasHeader = false
    static let definition = "string name\nstring level\n"

    let name: String
    let level: String

    init() {
        name = ""
        level = ""
    }

}

struct GetLoggersRequest: ServiceMessage {
    static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
    static let srvMd5sum = GetLoggers.md5sum
    static let srvDatatype = GetLoggers.datatype
    static let datatype = "roscpp/GetLoggersRequest"
    static let hasHeader = false
    static let definition = "\n"
}

struct GetLoggersResponse: ServiceMessage {
    static let md5sum = "32e97e85527d4678a8f9279894bb64b0"
    static let srvMd5sum = GetLoggers.md5sum
    static let srvDatatype = GetLoggers.datatype
    static let datatype = "roscpp/GetLoggersResponse"
    static let hasHeader = false
    static let definition = """
            Logger[] loggers

            ================================================================================\n\
            MSG: roscpp/Logger
            string name
            string level
            """

    let loggers: [Logger]

    init() {
        self.loggers = [Logger]()
    }
}

struct GetLoggers {
    typealias Request = GetLoggersRequest
    typealias Response = GetLoggersResponse
    static let md5sum = "32e97e85527d4678a8f9279894bb64b0"
    static let datatype = "roscpp/GetLoggers"
}

func getLoggers(x: GetLoggersRequest, y: inout GetLoggersRequest) -> Bool {
    ROS_ERROR("getLoggers not implemented")
    return false
}

struct SetLoggerLevelRequest: ServiceMessage {
    static let md5sum = "51da076440d78ca1684d36c868df61ea"
    static let datatype = "roscpp/SetLoggerLevelRequest"
    static let hasHeader = false
    static let definition = "string logger\nstring level\n"
    static let srvMd5sum = SetLoggerLevel.md5sum
    static let srvDatatype = SetLoggerLevel.datatype

    let logger: String
    let level: String

    init() {
        logger = ""
        level = ""
    }
}

struct SetLoggerLevelResponse: ServiceMessage {
    static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
    static let datatype = "roscpp/SetLoggerLevelResponse"
    static let hasHeader = false
    static let definition = "\n"
    static let srvMd5sum = SetLoggerLevel.md5sum
    static let srvDatatype = SetLoggerLevel.datatype
}

struct SetLoggerLevel {
    typealias Request = SetLoggerLevelRequest
    typealias Response = SetLoggerLevelResponse
    static let md5sum = "51da076440d78ca1684d36c868df61ea"
    static let datatype = "roscpp/SetLoggerLevel"
}

extension LoggerMessageType {

    init?(_ level: String) {
        switch level.lowercased() {
        case "debug":
            self = .debug
        case "info":
            self = .info
        case "warn":
            self = .warning
        case "error":
            self = .error
        case "fatal":
            self = .error
        default:
            return nil
        }
    }
}

func setLoggerLevel(x: SetLoggerLevelRequest) -> SetLoggerLevelResponse? {
    guard let level = LoggerMessageType(x.level) else {
        ROS_ERROR("Cannot set logger level \(x.level) for logger \(x.logger)")
        return nil
    }

    Console.setLoggerLevel(logger: x.logger, level: level)
    return SetLoggerLevelResponse()
}
