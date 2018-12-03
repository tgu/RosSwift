//
//  hashing.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-12.
//

import Foundation
#if os(macOS) || os(iOS) || os(tvOS) || os(watchOS)
import CommonCrypto
#elseif os(Linux)
import OpenSSL
typealias CC_LONG = size_t
#endif

extension Data {

    /// MD5 Hashing algorithm for hashing a Data instance.
    ///
    /// - Returns: The requested hash output or nil if failure.
    public func hashed() -> String {

        #if os(macOS) || os(iOS) || os(tvOS) || os(watchOS)
        let length = CC_MD5_DIGEST_LENGTH
        #elseif os(Linux)
        let length = MD5_DIGEST_LENGTH
        #endif

        var digest = Data(count: Int(length))

        // generate md5 hash
        _ = digest.withUnsafeMutableBytes { (digestBytes: UnsafeMutablePointer<UInt8>) in
            self.withUnsafeBytes { (messageBytes: UnsafePointer<UInt8>) in
                let length = CC_LONG(self.count)
                #if os(macOS) || os(iOS) || os(tvOS) || os(watchOS)
                CC_MD5(messageBytes, length, digestBytes)
                #elseif os(Linux)
                MD5(messageBytes, length, digestBytes)
                #endif
           }
        }

        // return the value based on the specified output type.
        return digest.map { String(format: "%02hhx", $0) }.joined()
    }
}


public extension String {

    /// MD5 Hashing algorithm for hashing a string instance.
    /// - Returns: The requested hash output or nil if failure.
    public func hashed() -> String? {

        // convert string to utf8 encoded data
        guard let message = data(using: .utf8) else { return nil }
        return message.hashed()
    }
}

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

    // Should not be hardcoded
    let env = ["PYTHONPATH":"/opt/ros/melodic/lib/python2.7/site-packages","ROS_PACKAGE_PATH":"/opt/ros/melodic/share"]

    return shell("/opt/ros/melodic/bin/rosmsg",cmd,env)
}

struct StdMessage<T> {
    var value : T

}

let types = ["int8": "Int8",
             "int16": "Int16",
             "int32": "Int32",
             "int64": "Int64",
             "uint8": "UInt8",
             "uint16": "UInt16",
             "uint32": "UInt32",
             "uint64": "UInt64",
             "string": "String",
             "byte": "Int8",
             "char": "UInt8",
             "duration": "RosTime.Duration",
             "time": "RosTime.TimeBase",
             "bool": "Bool",
             "float32": "Float32",
             "float64": "Float64",
             "empty": "Empty",
             "Header": "std_msgs.header"
            ]

var generatedMessages = [String:String]()

func generateCode(msg: String) {
    let data = rosmsg(["info",msg]).trimmingCharacters(in: .whitespacesAndNewlines)

    let parts = data.components(separatedBy: .whitespaces)
    let structName = parts[0] == "" ? "empty" : parts[0]
    let type = types[structName]!
    let name = parts.count > 1 ? String(parts[1]) : "data"
    let decl = "public var \(name): \(type)"
    let md5sum = String(data).hashed() ?? "*"

    let code = """
    import Foundation
    import RosTime

    extension std_msgs {
        public struct \(structName): Message {
            \(decl)
            public static var md5sum: String = "\(md5sum)"
            public static var datatype = "\(msg)"
            public static var definition = "\(data)"
            public static var hasHeader = false


            public init(_ value: \(type)) {
                self.\(name) = value
            }

        }
    }
    """

    let file = "Sources/StdMsgs/\(structName)Msg.swift"
    try? code.write(toFile: file, atomically: false, encoding: .utf8)
}

func generateMessageCode(msg: String) {
    struct dataItem {
        let name : String
        let simpleType : String
        let builtin : Bool
        let array : Bool
        let fixedArraySize : Int?
        let module : String?
        let value : String?
        var type : String {
            if array {
                return "[" + simpleType + "]"
            }
            return simpleType
        }

        var fullType: String {
            if let m = module {
                return m + "." + type
            }
            return type
        }

        var initCode: String? {
            return value == nil ? "self.\(name) = \(name)" : nil
        }

        var argument: String? {
            return value == nil ?  "\(name): \(fullType)" : nil
        }

        var codeInit: String? {
            return value == nil ?  "\(name) = \(fullType)()" : nil
        }

        var declaration: String {
            if let v = value {
                return "public let \(name): \(fullType) = \(v)"
            }
            return "public var \(name): \(fullType)"
        }

        init(name: String, type: String, value: String? = nil) {
            var isArray = false
            var isBuiltin = false
            var fixedArraySize : Int?
            let parts = type.components(separatedBy: "/")
            var t = parts.last!
            module = parts.count > 1 ? parts.first : nil
            if t.hasSuffix("[]") {
                t = String(t.dropLast(2))
                isArray = true
            } else if t.hasSuffix("]") {
                if let index = t.firstIndex(of: "[") {
                    let arraySizeStr = t.suffix(from: index).dropLast().dropFirst()
                    if let arraySize = Int(arraySizeStr) {
                        fixedArraySize = arraySize
                        isArray = true
                    }
                    t = String(t.prefix(upTo: index))
                }
            }
            if let typ = types[t] {
                t = typ
                isBuiltin = true
            }
            self.array = isArray
            self.name = name
            self.simpleType = t
            self.builtin = isBuiltin
            self.value = value
            self.fixedArraySize = fixedArraySize
        }
    }
    var items = [dataItem]()
    let messageType = msg.replacingOccurrences(of: "/", with: ".")

    let dataStr = rosmsg(["info","-r",msg]).trimmingCharacters(in: .whitespacesAndNewlines)
    let data = dataStr.components(separatedBy: .newlines).filter{ $0 != "" }

    for line in data {
        let parts = line.trimmingCharacters(in: .whitespaces).components(separatedBy: .whitespaces).filter { $0 != "" }
        if parts.count > 1 && !parts[0].hasPrefix("#") {
            let structName = parts[0]
            let name = parts[1]
            if parts.count > 2 && parts[2] == "=" {
                items.append(.init(name: name, type: structName, value: parts[3]))
            } else {
                items.append(.init(name: name,type: structName))
            }
        }
    }

    let decl = items.map{$0.declaration}.joined(separator: "\n")
    let md5sum = String(rosmsg(["md5",msg]).trimmingCharacters(in: .whitespacesAndNewlines))
    let arguments = items.compactMap{$0.argument}.joined(separator: ", ")
    let initCode = items.compactMap{ $0.initCode }.joined(separator: "\n")
    let codeInit = items.compactMap{$0.codeInit}.joined(separator: "\n")
    let path = messageType.components(separatedBy: ".")
    let modules = Set(items.compactMap{$0.module})
    let importModules = modules.map{"import \($0)"}.joined(separator: "\n")
    let hasHeader = "false"  // Some logic here...

    let comments = data.filter{ $0.starts(with: "#") }
        .joined(separator: "\n")
        .replacingOccurrences(of: "#", with: "///")
    var argInit = ""
    if !arguments.isEmpty {
        argInit = """
        public init(\(arguments)) {
        \(initCode)
        }
        """
    }

    let code = """
    import Foundation
    import StdMsgs
    import RosTime
    \(importModules)

    extension \(path.dropLast().joined(separator: ".")) {
    \(comments)
    public struct \(path.last!): Message {
    public static var md5sum: String = "\(md5sum)"
    public static var datatype = "\(msg)"
    public static var definition = \"\"\"
    \(dataStr)
    \"\"\"
    public static var hasHeader = \(hasHeader)

    \(decl)

    \(argInit)

    public init() {
        \(codeInit)
    }

    }
    }
    """
    let file = "Sources/msgs/\(messageType.replacingOccurrences(of: ".", with: "/"))Msg.swift"
    print("writing to \(file)" )
    try? code.write(toFile: file, atomically: false, encoding: .utf8)

}

let allMsgs = rosmsg(["list"]).components(separatedBy: .newlines)


let msgs = allMsgs.filter { !$0.hasPrefix("std_msgs") }
for msg in msgs {
    generateMessageCode(msg: msg)
}


let stdMsgs = allMsgs.filter { $0.hasPrefix("std_msgs") && !$0.contains("Array") }
for msg in stdMsgs {
    generateCode(msg: msg)
}









