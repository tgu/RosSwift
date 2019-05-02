//
//  filelog.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation

public struct FileLog {

    var logDirectory: String = ""

    public init(thisNodeName: String, remappings: StringStringMap) {
        // Log filename can be specified on the command line through __log
        // If it's been set, don't create our own name
        #if os(OSX) || os(Linux)
        var logFileName = remappings["__log"] ?? ""
        if logFileName == "" {
            if let rosLogDir = ProcessInfo.processInfo.environment["ROS_LOG_DIR"] {
                logFileName = rosLogDir + "/"
            } else {
                if let rosHome = ProcessInfo.processInfo.environment["ROS_HOME"] {
                    logFileName = rosHome + "/log/"
                } else {
                    if let home = ProcessInfo.processInfo.environment["HOME"] {
                        logFileName = home + "/.ros/log/"
                        do {
                            try FileManager.default.createDirectory(atPath: logFileName,
                                                                    withIntermediateDirectories: true)
                        } catch {
                            ROS_ERROR("Could not create log directory \(logFileName), reason: \(error)")
                        }
                    }
                }
            }

            // sanitize the node name and tack it to the filename
            logFileName = logFileName.components(separatedBy: "/").map {
                $0.components(separatedBy: CharacterSet.urlPathAllowed.inverted).joined(separator: "_")
            }.joined(separator: "/")

            logDirectory = logFileName + thisNodeName.dropFirst()
            logFileName += thisNodeName.dropFirst() + "/\(getpid()).log"
        }
        #elseif os(iOS) || os(tvOS) || os(watchOS)
            // Do something else here
        #endif

//            log_file_name = fs::system_complete(log_file_name).string();
//            g_log_directory = fs::path(log_file_name).parent_path().string();


    }

}
