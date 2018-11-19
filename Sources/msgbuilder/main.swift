//
//  hashing.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-12.
//

import Foundation
import CommonCrypto

extension Data {

    /// Hashing algorithm that prepends an RSA2048ASN1Header to the beginning of the data being hashed.
    ///
    /// - Parameters:
    ///   - type: The type of hash algorithm to use for the hashing operation.
    ///   - output: The type of output string desired.
    /// - Returns: A hash string using the specified hashing algorithm, or nil.
    public func hashWithRSA2048Asn1Header(_ type: HashType, output: HashOutputType = .hex) -> String? {

        let rsa2048Asn1Header:[UInt8] = [
            0x30, 0x82, 0x01, 0x22, 0x30, 0x0d, 0x06, 0x09, 0x2a, 0x86, 0x48, 0x86,
            0xf7, 0x0d, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0f, 0x00
        ]

        var headerData = Data(bytes: rsa2048Asn1Header)
        headerData.append(self)

        return hashed(type, output: output)
    }

    /// Hashing algorithm for hashing a Data instance.
    ///
    /// - Parameters:
    ///   - type: The type of hash to use.
    ///   - output: The type of hash output desired, defaults to .hex.
    ///   - Returns: The requested hash output or nil if failure.
    public func hashed(_ type: HashType, output: HashOutputType = .hex) -> String? {

        // setup data variable to hold hashed value
        var digest = Data(count: Int(type.length))

        // generate hash using specified hash type
        _ = digest.withUnsafeMutableBytes { (digestBytes: UnsafeMutablePointer<UInt8>) in
            self.withUnsafeBytes { (messageBytes: UnsafePointer<UInt8>) in
                let length = CC_LONG(self.count)
                switch type {
                case .md5: CC_MD5(messageBytes, length, digestBytes)
                case .sha1: CC_SHA1(messageBytes, length, digestBytes)
                case .sha224: CC_SHA224(messageBytes, length, digestBytes)
                case .sha256: CC_SHA256(messageBytes, length, digestBytes)
                case .sha384: CC_SHA384(messageBytes, length, digestBytes)
                case .sha512: CC_SHA512(messageBytes, length, digestBytes)
                }
            }
        }

        // return the value based on the specified output type.
        switch output {
        case .hex: return digest.map { String(format: "%02hhx", $0) }.joined()
        case .base64: return digest.base64EncodedString()
        }
    }
}

// Defines types of hash string outputs available
public enum HashOutputType {
    // standard hex string output
    case hex
    // base 64 encoded string output
    case base64
}

// Defines types of hash algorithms available
public enum HashType {
    case md5
    case sha1
    case sha224
    case sha256
    case sha384
    case sha512

    var length: Int32 {
        switch self {
        case .md5: return CC_MD5_DIGEST_LENGTH
        case .sha1: return CC_SHA1_DIGEST_LENGTH
        case .sha224: return CC_SHA224_DIGEST_LENGTH
        case .sha256: return CC_SHA256_DIGEST_LENGTH
        case .sha384: return CC_SHA384_DIGEST_LENGTH
        case .sha512: return CC_SHA512_DIGEST_LENGTH
        }
    }
}

public extension String {

    /// Hashing algorithm for hashing a string instance.
    ///
    /// - Parameters:
    ///   - type: The type of hash to use.
    ///   - output: The type of output desired, defaults to .hex.
    /// - Returns: The requested hash output or nil if failure.
    public func hashed(_ type: HashType, output: HashOutputType = .hex) -> String? {

        // convert string to utf8 encoded data
        guard let message = data(using: .utf8) else { return nil }
        return message.hashed(type, output: output)
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
    let md5sum = String(data).hashed(.md5) ?? "*"
    let size = structName == "string" ? "4 + data.utf8.count" : "MemoryLayout<\(type)>.size"

    let ser = """
    var buffer = StreamBuffer()
                let len = \(size)
                StdMsgs.serialize(stream: &buffer, t: UInt32(len))
                StdMsgs.serialize(stream: &buffer, t: \(name))
                return SerializedMessage(msg: self, buffer: buffer.buffer)
    """

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

            public init() {
                self.\(name) = \(type)()
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

        var size : String? {
            if value != nil {
                return nil
            }
            if array {
                if builtin {
                    if let fs = fixedArraySize {
                        return "MemoryLayout<\(simpleType)>.size * \(fs)"
                    }
                    return "4 + MemoryLayout<\(simpleType)> * \(name).count"
                } else {
                    return "\(name).reduce(0, { $0+$1.serializationLength() })"
                }
            }
            return "\(name).serializationLength()"
        }

        var ser : String? {
            if value != nil {
                return nil
            }
            if builtin && simpleType != "std_msgs.header" {
                return "StdMsgs.serialize(stream: &stream, t: \(name))"
            }
            if array {
                if fixedArraySize == nil {
                    return """
                    \(name).forEach{ $0.serialize(stream: &stream) }
                    """
                }
                return "\(name).forEach{ $0.serialize(stream: &stream) }"
            }
            return "\(name).serialize(stream: &stream)"


        }

        var deser : String? {
            if value != nil {
                return nil
            }
            if array {
                return """
                var list = [\(simpleType)]()
                for _ in 0..<\(name).count {
                    var item = \(simpleType)()
                    item.deserialize(from: &from )
                    list.append(item)
                }
                \(name) = list
                """
            }
            return "\(name).deserialize(from: &from)"

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

generateMessageCode(msg: "actionlib_msgs/GoalStatus")

let allMsgs = rosmsg(["list"]).components(separatedBy: .newlines)
let msgs = allMsgs.filter { !$0.hasPrefix("std_msgs") }
for msg in msgs {
    generateMessageCode(msg: msg)
}

let stdMsgs = allMsgs.filter { $0.hasPrefix("std_msgs") && !$0.contains("Array") }
for msg in stdMsgs {
    generateCode(msg: msg)
}








