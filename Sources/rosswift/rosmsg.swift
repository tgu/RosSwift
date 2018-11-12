//
//  rosmsg.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-12.
//

import Foundation

func rosmsg(_ cmd: [String]) -> String {
    func shell(_ command: String, _ args: [String], _ environment: [String:String] = [:]) -> String {
        let task = Process()
        task.launchPath = command
        task.arguments = args
        task.environment = environment
        let pipe = Pipe()
        task.standardOutput = pipe
        task.launch()
        task.waitUntilExit()
        let data = pipe.fileHandleForReading.readDataToEndOfFile()

        guard let stringRead = String(data: data, encoding: .utf8 ) else {
            return ""
        }

        return stringRead
    }

    let env = ["PYTHONPATH":"/opt/ros/melodic/lib/python2.7/site-packages","ROS_PACKAGE_PATH":"/opt/ros/melodic/share"]

    return shell("/opt/ros/melodic/bin/rosmsg",cmd,env)
}
