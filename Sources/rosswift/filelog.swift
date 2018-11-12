//
//  filelog.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation


public struct FileLog {
    public init(remappings: M_string)
    {
        // Log filename can be specified on the command line through __log
        // If it's been set, don't create our own name
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
                        try! FileManager.default.createDirectory(atPath: logFileName, withIntermediateDirectories: true)
                    }
                }
            }

            // sanitize the node name and tack it to the filename
            logFileName.components(separatedBy: CharacterSet.alphanumerics.inverted).joined(separator: "_")
            logFileName = logFileName + Ros.this_node.getName() + "_\(getpid()).log"
        }


//            log_file_name = fs::system_complete(log_file_name).string();
//            g_log_directory = fs::path(log_file_name).parent_path().string();

    }

}
