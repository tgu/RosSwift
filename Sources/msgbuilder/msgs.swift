//
//  msgs.swift
//  msgbuilder
//
//  Created by Thomas Gustafsson on 2019-04-29.
//

import Foundation

let HEADER = "Header"
let HEADER_FULL_NAME = "std_msgs/Header"

let TIME     = "time"
let DURATION = "duration"
let PRIMITIVE_TYPES = ["int8","uint8","int16","uint16","int32","uint32","int64",
                       "uint64","float32","float64", "string", "bool", "char","byte"]
let BUILTIN_TYPES = PRIMITIVE_TYPES + [TIME, DURATION]

protocol FullType {
    var field_type: String { get }
    var isArray: Bool { get }
    func fullType(in package: String) -> String
}

extension FullType {
    func fullType(in package: String) -> String {
        let msg_type = bare_msg_type(field_type)

        var type = ""
        if let (pack,base) = package_resource_name(name: msg_type) {
            if pack == package {
                type = base
            } else if pack.isEmpty {
                type = swiftType[base] ?? base
            } else {
                type = pack + "." + base
            }
        } else {
            type = msg_type.replacingOccurrences(of: "/", with: ".")
        }
        return isArray ? "[\(type)]" : type
    }
}

struct Constant: FullType {
    let field_type: String
    let name: String
    let val_converted: Any
    let val: String
    let isArray: Bool = false

    init?(line: String) {
        let clean_line = strip_comments(line)
        let line_splits = clean_line.components(separatedBy: " ").filter { !$0.isEmpty }
        let field_type = line_splits[0]
        var name = ""
        var val = ""
        guard PRIMITIVE_TYPES.contains(field_type) else {
            print("\(field_type) is not a legal constant type")
            return nil
        }
        if field_type == "string" {
            let idx = line.firstIndex(of: "=")!
            name = String(line.prefix(upTo: idx))
            val = String(line.suffix(from: idx).dropFirst())
        } else {
            let splits = line_splits
                .dropFirst()
                .map{ $0.trimmingCharacters(in: .whitespaces)}
                .joined(separator: " ")
                .components(separatedBy: "=")
            guard splits.count == 2 else {
                print("Invalid constant declaration: \(line)")
                return nil
            }
            name = splits[0]
            val = splits[1].trimmingCharacters(in: .whitespaces)
        }
        guard let val_converted = convert_constant_value(field_type: field_type, val: val) else {
            print("Invalid constant value: \(val)")
            return nil
        }

        self.field_type = field_type
        self.name = name
        self.val = val.trimmingCharacters(in: .whitespaces)
        self.val_converted = val_converted
    }

    func declaration(in package: String) -> String {
        return "\tpublic static let \(name): \(fullType(in: package)) = \(val)"
    }
}

struct Variable: FullType {
    let field_type: String
    let name: String

    init?(orig_line: String, package_context: String) {
        let clean_line = strip_comments(orig_line).trimmingCharacters(in: .whitespaces)
        if clean_line.isEmpty {
            return nil
        }
        let line_splits = clean_line.components(separatedBy: " ").filter { !$0.isEmpty }
        guard line_splits.count == 2 else {
            print("Invalid declaration: \(orig_line)")
            return nil
        }
        var field_type = line_splits[0]
        var name = line_splits[1]
        guard is_valid_msg_field_name(name) else {
            print("\(name) is not a legal message field name")
            return nil
        }
        guard is_valid_msg_type(field_type) else {
            print("\(field_type) is not a legal message field type")
            return nil
        }
        if !package_context.isEmpty && !field_type.contains("/") {
            if field_type == HEADER {
                field_type = HEADER_FULL_NAME
            } else if !is_builtin(bare_msg_type(field_type)) {
                field_type = "\(package_context)/\(field_type)"
            }
        } else if field_type == HEADER {
            field_type = HEADER_FULL_NAME
        }

        // Check for reserved names

        if ["md5sum","datatype","hasHeader","definition"].contains(name) {
            name = "_" + name
        }


        self.field_type = field_type
        self.name = name
    }


    func declaration(in package: String) -> String {
        return "\tpublic var \(name): \(fullType(in: package))"
    }

    func argument(in package: String) -> String {
        return "\(name): \(fullType(in: package))"
    }

    var isArray: Bool {
        return field_type.contains("[")
    }

    var size: String? {
        if let fb = field_type.firstIndex(of: "["), let _ = field_type.firstIndex(of: "]") {
            let size = String(field_type.suffix(from: fb).dropFirst().dropLast()).trimmingCharacters(in: .whitespaces)
            if size.isEmpty {
                return nil
            }
            return size
        }
        return nil
    }

    var initCode: String {
        if let size = size {
            return "\t\tassert(\(name).count == \(size))\n\t\t\tself.\(name) = \(name)"
        }
        return "\t\tself.\(name) = \(name)"
    }

    func codeInit(in package: String) -> String {
        if let size = size {
            return "\t\t\(name) = \(fullType(in: package))(repeating: 0, count: \(size))"
        } else {
            return "\t\t\(name) = \(fullType(in: package))()"
        }
    }

    var module: String? {
        if let (package,_) = package_resource_name(name: field_type) {
            if package.isEmpty {
                return nil
            }
            return package
        }
        return nil
    }

    var simpleType: String {
        let type = bare_msg_type(field_type)
        return type.replacingOccurrences(of: "/", with: ".")
    }

}

struct MsgSpec {
    let variables: [Variable]
    let constants: [Constant]
    let text: String
    let full_name: String
    let package: String
    let short_name: String

    var messageType: String {
        return full_name.replacingOccurrences(of: "/", with: ".")
    }
    
    init?(text: String, full_name: String) {
        guard let (package_name, short_name) = package_resource_name(name: full_name) else {
            return nil
        }

        var constants = [Constant]()
        var vars = [Variable]()

        for orig_line in text.components(separatedBy: .newlines) {
            let clean_line = strip_comments(orig_line)
            if clean_line.isEmpty {
                continue
            }
            if clean_line.contains("=") {
                if let const = Constant(line: orig_line) {
                    constants.append(const)
                }
            } else {
                if let variable = Variable(orig_line: orig_line, package_context: package_name) {
                    vars.append(variable)
                }
            }
        }
        self.variables = vars
        self.constants = constants
        self.text = text
        self.full_name = full_name
        self.package = package_name
        self.short_name = short_name
    }

    func compute_md5_text(msg_context: MsgContext) -> String? {
        var buff = constants.map { "\($0.field_type) \($0.name)=\($0.val)"}.joined(separator: "/n")
        for v in variables {
            let msg_type = bare_msg_type(v.field_type)
            if is_builtin(msg_type) {
                buff += "\(v.field_type) \(v.name)\n"
            } else {
                let sub_spec = msg_context.get_registered(msg_type: msg_type)
                guard let sub_md5 = sub_spec?.compute_md5(msg_context: msg_context) else {
                    return nil
                }
                buff += "\(sub_md5) \(v.name)\n"
            }
        }
        return buff.trimmingCharacters(in: .newlines)
    }

    func compute_md5(msg_context: MsgContext) -> String? {
        return compute_md5_text(msg_context: msg_context)?.hashed()
    }
}

func is_valid_msg_field_name(_ x: String) -> Bool {
    return is_legal_resource_base_name(x)
}

func is_valid_msg_type(_ x: String) -> Bool {
    return true
}

func bare_msg_type(_ x: String) -> String {
    if let index = x.firstIndex(of: "[") {
        return String(x.prefix(upTo: index))
    }
    return x
}

func is_builtin(_ x: String) -> Bool {
    return BUILTIN_TYPES.contains(x)
}

func resolve_type(_ msg_type: String, package_context: String) -> String {
    let bt = bare_msg_type(msg_type)
    if BUILTIN_TYPES.contains(bt) {
        return msg_type
    } else if bt == HEADER {
        return HEADER_FULL_NAME
    } else if msg_type.contains("/") {
        return msg_type
    } else {
        return "\(package_context)/\(msg_type)"
    }
}
