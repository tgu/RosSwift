//
//  rosconsole.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-10.
//

import Foundation
import Logging
import Atomics
import Synchronization

let rosPackageName: String? = nil

let rosConsoleRootLoggerName = "ros"
let rosConsolePacakgeName = rosPackageName ?? "unknown_package"
let rosConsoleNamePrefix = rosConsoleRootLoggerName + "/" + (rosConsolePacakgeName)
let rosConsoleDefaultName = rosConsoleNamePrefix

/// Backing store for the library logger. Its level starts at the build
/// configuration (debug in DEBUG builds, info otherwise) and can be changed at
/// runtime via the `~set_logger_level` service (see `Console.setLoggerLevel`).
private let loggerState = Mutex<Logging.Logger>({
    var l = Logging.Logger(label: rosConsoleRootLoggerName)
#if DEBUG
    l.logLevel = .debug
#else
    l.logLevel = .info
#endif
    return l
}())

/// A snapshot of the library's swift-log logger. All `ROS_*` logging helpers
/// route through it, so applications only need to bootstrap a swift-log
/// backend (or rely on the default `StreamLogHandler`).
var logger: Logging.Logger { loggerState.withLock { $0 } }

protocol LogAppender {

}

func rosVerbose(_ msg: @autoclosure () -> String,
                functionName: String = #function,
                lineNum: Int = #line,
                fileName: String = #file) {

    logger.trace("\(msg())", file: fileName, function: functionName, line: UInt(lineNum))
}

func ROS_DEBUG(_ msg: @autoclosure () -> String,
               functionName: String = #function,
               lineNum: Int = #line,
               fileName: String = #file) {

#if DEBUG
    logger.debug("\(msg())", file: fileName, function: functionName, line: UInt(lineNum))
#endif
}

public func ROS_WARN_STREAM(_ msg: @autoclosure () -> String,
                            functionName: String = #function,

                            lineNum: Int = #line,
                            fileName: String = #file) {
    logger.warning("\(msg())", file: fileName, function: functionName, line: UInt(lineNum))
}

public func ROS_ERROR_STREAM(_ msg: @autoclosure () -> String,
                             functionName: String = #function,

                             lineNum: Int = #line,
                             fileName: String = #file) {
    logger.error("\(msg())", file: fileName, function: functionName, line: UInt(lineNum))
}

public func ROS_INFO_STREAM(_ msg: @autoclosure () -> String,
                            functionName: String = #function,

                            lineNum: Int = #line,
                            fileName: String = #file) {
    logger.info("\(msg())", file: fileName, function: functionName, line: UInt(lineNum))
}

public func ROS_WARN_STREAM_THROTTLE(_ timeout: Double, _ msg: @autoclosure () -> String,
                                     functionName: String = #function,

                                     lineNum: Int = #line,
                                     fileName: String = #file) {
    logger.warning("\(msg())", file: fileName, function: functionName, line: UInt(lineNum))
}



public func ROS_INFO(_ msg: @autoclosure () -> String,
                     functionName: String = #function,

                     lineNum: Int = #line,
                     fileName: String = #file) {
    logger.info("\(msg())", file: fileName, function: functionName, line: UInt(lineNum))
}

public func ROS_ERROR(_ msg: @autoclosure () -> String,
                      functionName: String = #function,
                      lineNum: Int = #line,
                      fileName: String = #file) {
    logger.error("\(msg())", file: fileName, function: functionName, line: UInt(lineNum))
}

public func ROS_WARNING(_ msg: @autoclosure () -> String,
                        functionName: String = #function,
                        lineNum: Int = #line,
                        fileName: String = #file) {

    logger.warning("\(msg())", file: fileName, function: functionName, line: UInt(lineNum))
}

func ROS_DEBUG_NAMED(_ name: String, _ text: String) {
    Console.printDebug("\(name): \(text)")
}

func ROS_LOG_DEBUG(_ text: String) {
    Console.printDebug("rosswift_internal: \(text)")
}

func ROS_LOG_ERROR(_ text: String) {
    Console.print("rosswift_internal: \(text)")
}

internal struct Console {
    static let gInitialized = ManagedAtomic(false)
    static let gExtraFixedTokens = Mutex(StringStringMap())

    static func initialize() {

    }

    static func setFixedFilterToken(key: String, val: String) {
        gExtraFixedTokens.withLock { $0[key] = val }
    }

    static func registerAppender(appender: LogAppender) {
        logger.debug("register_appender not implemented")
    }

    static func printDebug(_ text: String) {
        logger.debug("\(text)")
    }

    static func print(_ text: String) {
        ROS_INFO(text)
    }

    static func setLoggerLevel(logger name: String, level: Logging.Logger.Level) {
        loggerState.withLock { $0.logLevel = level }
        ROS_INFO("set logger [\(name)] level to \(level)")
    }

}
