//
//  logger.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-11-19.
//

import StdMsgs
import LoggerAPI


struct Logger: Message {

    static let md5sum = "a6069a2ff40db7bd32143dd66e1f408e"
    static let datatype = "roscpp/Logger"
    static let definition = "string name\nstring level\n"

    let name: String
    let level: String

    init() {
        name = ""
        level = ""
    }

}



struct GetLoggers: ServiceProt {
    static let md5sum = "32e97e85527d4678a8f9279894bb64b0"
    static let datatype = "roscpp/GetLoggers"
    var request: Request
    var response: Response

    struct Request: ServiceRequestMessage {
        public typealias ServiceType = GetLoggers
        static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
        static let datatype = "roscpp/GetLoggersRequest"
        static let definition = "\n"
    }

    struct Response: ServiceResponseMessage {
        public typealias ServiceType = GetLoggers
        static let md5sum = "32e97e85527d4678a8f9279894bb64b0"
        static let datatype = "roscpp/GetLoggersResponse"
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
}

func getLoggers(x: GetLoggers.Request, y: inout GetLoggers.Request) -> Bool {
    ROS_ERROR("getLoggers not implemented")
    return false
}

struct SetLoggerLevel: ServiceProt {
    static let md5sum = "51da076440d78ca1684d36c868df61ea"
    static let datatype = "roscpp/SetLoggerLevel"
    var request: Request
    var response: Response

    struct Request: ServiceRequestMessage {
        public typealias ServiceType = SetLoggerLevel
        static let md5sum = "51da076440d78ca1684d36c868df61ea"
        static let datatype = "roscpp/SetLoggerLevelRequest"
        static let definition = "string logger\nstring level\n"

        let logger: String
        let level: String

        init() {
            logger = ""
            level = ""
        }
    }

    struct Response: ServiceResponseMessage {
        public typealias ServiceType = SetLoggerLevel
        static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
        static let datatype = "roscpp/SetLoggerLevelResponse"
        static let definition = "\n"
    }


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

func setLoggerLevel(x: SetLoggerLevel.Request) -> SetLoggerLevel.Response? {
    guard let level = LoggerMessageType(x.level) else {
        ROS_ERROR("Cannot set logger level \(x.level) for logger \(x.logger)")
        return nil
    }

    Console.setLoggerLevel(logger: x.logger, level: level)
    return SetLoggerLevel.Response()
}
