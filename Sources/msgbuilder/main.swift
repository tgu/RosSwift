//
//  hashing.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-12.
//

import Foundation

let usage = """
msgbuilder is a tool for converting ros msg files to swift structs

Usage:
msgbuilder [-o] source destination
msgbuilder [-o] -r destination

options:
    -o overwrite existing files
    -r convert all messages found by 'rosmsg list'

source is a directory with message definitions
destination is the directory where the swift message files will be stored
"""

extension URL {
    var isDirectory: Bool {
        var isDir: ObjCBool = false
        if FileManager.default.fileExists(atPath: self.path, isDirectory: &isDir) {
            if isDir.boolValue {
                return true
            }
        }
        return false
    }
}

let options = CommandLine.arguments.filter { $0.hasPrefix("-")}
let args = CommandLine.arguments.dropFirst().filter { !$0.hasPrefix("-")}

var overwrite = false
var useRosMsg = false
let context = MsgContext()

for opt in options {
    switch opt {
    case "-o":
        overwrite = true
    case "-r":
        useRosMsg = true
    default:
        print("unknown option \(opt)")
        exit(1)
    }
}

if useRosMsg {
    if args.count != 1 {
        print(usage)
        exit(0)
    }

    let destination = URL(fileURLWithPath: args[0])
    let parent = destination.deletingLastPathComponent()
    if !parent.isDirectory {
        print("\(parent) is not a valid directory")
        exit(0)
    }

    if let shell = Shell() {
        let allMsgsString = shell.rosmsg(["list"]).trimmingCharacters(in: .newlines)
        let allMsgs = allMsgsString.components(separatedBy: .newlines)

        for msg in allMsgs {
            let content = shell.rosmsg(["info","-r",msg]).trimmingCharacters(in: .whitespacesAndNewlines)
            print("getting info for '\(msg)'")
            _ = context.addMsg(with: content, full_name: msg)
        }
    }
    context.genAllMessages(to: destination)
    
    exit(0)
}

if args.count != 2 {
    print(usage)
    exit(0)
}

let source = URL(fileURLWithPath: args[0])
if !source.isDirectory {
    print("\(source) is not a valid directory")
    exit(0)
}

let destination = URL(fileURLWithPath: args[1])
let parent = destination.deletingLastPathComponent()
if !parent.isDirectory {
    print("\(parent) is not a valid directory")
    exit(0)
}

if !overwrite && FileManager.default.fileExists(atPath: destination.path) {
    print("A file already exist at \(destination.path)")
    exit(0)
}


var packageName = ""
let last = source.lastPathComponent
if last.hasSuffix("_msgs") {
    packageName = last
}

context.load_dir(path: source, package_name: packageName)
context.genAllMessages(to: destination)












