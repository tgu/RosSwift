//
//  rosconsole.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-10.
//

import Foundation
import LoggerAPI

let ROS_PACKAGE_NAME : String? = nil

let ROSCONSOLE_ROOT_LOGGER_NAME = "ros"
let ROSCONSOLE_PACKAGE_NAME = ROS_PACKAGE_NAME ?? "unknown_package"
let ROSCONSOLE_NAME_PREFIX = ROSCONSOLE_ROOT_LOGGER_NAME + "/" + (ROSCONSOLE_PACKAGE_NAME)
let ROSCONSOLE_DEFAULT_NAME = ROSCONSOLE_NAME_PREFIX

protocol LogAppender {
    
}

func ROS_VERBOSE(_ msg: @autoclosure () -> String, functionName: String = #function,
               lineNum: Int = #line, fileName: String = #file) {
    Log.verbose(msg, functionName: functionName, lineNum: lineNum, fileName: fileName)
}



func ROS_DEBUG(_ msg: @autoclosure () -> String, functionName: String = #function,
               lineNum: Int = #line, fileName: String = #file) {
    #if DEBUG
    Log.debug(msg, functionName: functionName, lineNum: lineNum, fileName: fileName)
    #endif
}

func ROS_INFO(_ msg: @autoclosure () -> String, functionName: String = #function,
               lineNum: Int = #line, fileName: String = #file) {
    Log.info(msg, functionName: functionName, lineNum: lineNum, fileName: fileName)
}


public func ROS_ERROR(_ msg: @autoclosure () -> String, functionName: String = #function,
                      lineNum: Int = #line, fileName: String = #file) {
    Log.error(msg, functionName: functionName, lineNum: lineNum, fileName: fileName)
}

public func ROS_WARNING(_ msg: @autoclosure () -> String, functionName: String = #function,
                      lineNum: Int = #line, fileName: String = #file) {
    Log.warning(msg, functionName: functionName, lineNum: lineNum, fileName: fileName)
}

func ROS_DEBUG_NAMED(_ name: String, _ text: String) {
    Ros.console.printDebug("\(name): \(text)")
}

func ROS_LOG_DEBUG(_ text: String) {
    Ros.console.printDebug("rosswift_internal: \(text)")
}

func ROS_LOG_ERROR(_ text: String) {
    Ros.console.print("rosswift_internal: \(text)")
}




extension Ros {

    struct console {
        static var g_initialized = false
        static var g_extra_fixed_tokens = M_string()
        static var g_locations_mutex = DispatchQueue(label: "location_mutex")
//        static var ros_logger = getLogger(ROSCONSOLE_ROOT_LOGGER_NAME)

        static func initialize() {

            
        }

        static func setFixedFilterToken(key: String, val: String) {
            g_extra_fixed_tokens[key] = val
        }

        static func register_appender(appender: LogAppender) {
            Log.error("register_appender not implemnted")
        }

        static func printDebug(_ text: String) {
            g_locations_mutex.sync {
                Log.debug(text)
            }
        }

        static func print(_ text: String) {
            g_locations_mutex.sync {
                ROS_INFO(text)
            }
        }

    }

}

