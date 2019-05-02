//
//  rosconsole.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-10.
//

import Foundation
import LoggerAPI

let rosPackageName: String? = nil

let rosConsoleRootLoggerName = "ros"
let rosConsolePacakgeName = rosPackageName ?? "unknown_package"
let rosConsoleNamePrefix = rosConsoleRootLoggerName + "/" + (rosConsolePacakgeName)
let rosConsoleDefaultName = rosConsoleNamePrefix

protocol LogAppender {

}

func rosVerbose(_ msg: @autoclosure () -> String,
                functionName: String = #function,
                lineNum: Int = #line,
                fileName: String = #file) {

    Log.verbose(msg(), functionName: functionName, lineNum: lineNum, fileName: fileName)
}

func ROS_DEBUG(_ msg: @autoclosure () -> String,
               functionName: String = #function,
               lineNum: Int = #line,
               fileName: String = #file) {

    #if DEBUG
    Log.debug(msg(), functionName: functionName, lineNum: lineNum, fileName: fileName)
    #endif
}

func ROS_INFO(_ msg: @autoclosure () -> String,
              functionName: String = #function,

              lineNum: Int = #line,
              fileName: String = #file) {
    Log.info(msg(), functionName: functionName, lineNum: lineNum, fileName: fileName)
}

public func ROS_ERROR(_ msg: @autoclosure () -> String,
                      functionName: String = #function,
                      lineNum: Int = #line,
                      fileName: String = #file) {

    Log.error(msg(), functionName: functionName, lineNum: lineNum, fileName: fileName)
}

public func ROS_WARNING(_ msg: @autoclosure () -> String,
                        functionName: String = #function,
                        lineNum: Int = #line,
                        fileName: String = #file) {

    Log.warning(msg(), functionName: functionName, lineNum: lineNum, fileName: fileName)
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
    static var gInitialized = false
    static var gExtraFixedTokens = StringStringMap()
    static var gLocationsQueue = DispatchQueue(label: "location_mutex")

    static func initialize() {

    }

    static func setFixedFilterToken(key: String, val: String) {
        gExtraFixedTokens[key] = val
    }

    static func registerAppender(appender: LogAppender) {
        Log.error("register_appender not implemnted")
    }

    static func printDebug(_ text: String) {
        gLocationsQueue.sync {
            Log.debug(text)
        }
    }

    static func print(_ text: String) {
        gLocationsQueue.sync {
            ROS_INFO(text)
        }
    }

    static func setLoggerLevel(logger: String, level: LoggerMessageType) {
        ROS_INFO("setLoggerLevel not implemented")
    }

}
