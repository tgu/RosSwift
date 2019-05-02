//
//  msg_loader.swift
//  msgbuilder
//
//  Created by Thomas Gustafsson on 2019-04-29.
//

import Foundation
import StdMsgs

class MsgContext {
    var registered: [String: [String: MsgSpec] ]
    var files: [String: String]
    var dependencies: [String: [MsgSpec]]

    init() {
        registered = [:]
        files = [:]
        dependencies = [:]

        // used in common_msgs and needed for md5 computation
        
        register(message: std_msgs.Header.self)
        register(message: std_msgs.ColorRGBA.self)
    }

    func register(message: Message.Type) {
        _ = addMsg(with: message.definition, full_name: message.datatype)
    }

    func register(full_msg_type: String, msgspec: MsgSpec) {
        if let (package, base_type) = package_resource_name(name: bare_msg_type(full_msg_type)) {
            if var pack = registered[package] {
                pack.updateValue(msgspec, forKey: base_type)
                registered[package] = pack
            } else {
                registered.updateValue([base_type: msgspec], forKey: package)
            }
        }
    }

    func get_registered(msg_type: String) -> MsgSpec? {
        let full_msg_type = bare_msg_type(msg_type)
        guard let (package, base_type) = package_resource_name(name: full_msg_type) else {
            return nil
        }
        return registered[package]?[base_type]
    }

    func is_registered(_ msg_type: String) -> Bool {
        let full_type = bare_msg_type(msg_type)
        if let (package, base_type) = package_resource_name(name: full_type) {
            return registered[package]?.keys.contains(base_type) ?? false
        }
        return false
    }

    func set_depends(full_msg_type: String, dependencies: [MsgSpec]) {
        self.dependencies[full_msg_type] = dependencies
    }

    func get_depends(full_msg_type: String) -> [MsgSpec]? {
        return dependencies[full_msg_type]
    }

    func set_file(full_msg_type: String, file_path: String) {
        files[full_msg_type] = file_path
    }

    func get_file(full_msg_type: String) -> String? {
        return files[full_msg_type]
    }

    func load_dir(path: URL, package_name: String) {
        print("===== \(path.path)")
        guard let content = try? FileManager.default.contentsOfDirectory(atPath: path.path) else {
            print("no files in directory \(path)")
            exit(1)
        }

        let packages = content.filter { $0.hasSuffix("_msgs")}
        let files = content.filter { $0.hasSuffix(".msg") }
//        let other = content.filter { !$0.hasSuffix("_msgs") && !$0.hasSuffix(".msg")}

        for file in files {
            let name = String(URL(fileURLWithPath: file).lastPathComponent.dropLast(4))
            let full_name = package_name + "/" + name
            let full_path = path.appendingPathComponent(file)
            print("load \(full_path.path)")
            _ = loadMsg(from: full_path.path, full_name: full_name)
        }


        if !packages.isEmpty {
            for package in packages {
                let sub_path = path.appendingPathComponent(package)
                load_dir(path: sub_path, package_name: package)
            }
        } else if content.contains("msg") {
            let sub_path = path.appendingPathComponent("msg")
            load_dir(path: sub_path, package_name: package_name)
        }
    }

    func addMsg(with content: String, full_name: String) -> MsgSpec? {
        if let spec = MsgSpec(text: content, full_name: full_name) {
            register(full_msg_type: full_name, msgspec: spec)
            return spec
        } else {
            print("\(full_name) = no spec")
            return nil
        }
    }

    func loadMsg(from path: String, full_name: String) -> MsgSpec? {
        if let content = try? String(contentsOfFile: path) {
            if let spec = addMsg(with: content, full_name: full_name) {
                set_file(full_msg_type: full_name, file_path: path)
                return spec
            }
        } else {
            print("Could not open \(path)")
        }
        return nil
    }

    func load_msg_by_type(msg_type: String, search_path: [String: [String]]) -> MsgSpec? {
        var type = msg_type
        if msg_type == HEADER {
            type = HEADER_FULL_NAME
        }
        if let (package, base_type) = package_resource_name(name: type) {
            if let file_path = get_msg_file(package: package, base_type: base_type, search_path: search_path) {
                let spec = loadMsg(from: file_path, full_name: type)
                return spec
            }
        }
        return nil
    }

    func load_msg_depends(spec: MsgSpec, search_path: [String: [String]]) -> [MsgSpec] {
        let package_context = spec.package
        var depends = [MsgSpec]()
        for unresolved_type in spec.variables.map({$0.field_type}) {
            let bare_type = bare_msg_type(unresolved_type)
            let resolved_type = resolve_type(bare_type, package_context: package_context)
            if is_builtin(resolved_type) {
                continue
            }
            var depspec: MsgSpec?
            if is_registered(resolved_type) {
                depspec = get_registered(msg_type: resolved_type)!
            } else {
                depspec = load_msg_by_type(msg_type: resolved_type, search_path: search_path)!
            }

            if let depspec = depspec {
                depends.append(depspec)
                if !dependencies.keys.contains(resolved_type) {
                    _ = load_msg_depends(spec: depspec, search_path: search_path)
                }
            }

        }
        set_depends(full_msg_type: spec.full_name, dependencies: depends)
        return depends
    }

    func genAllMessages(to destination: URL) {
        for package in registered.keys {
            let code = """
            // Generated by msgbuilder \(Date())

            public enum \(package) {}
            """

            let file = "\(destination.path)/\(package)/\(package).swift"
            let url = URL(fileURLWithPath: file)
            try? FileManager.default.createDirectory(at: url.deletingLastPathComponent(), withIntermediateDirectories: true)
            try? code.write(toFile: file, atomically: false, encoding: .utf8)
        }

        for (_,element) in registered.enumerated() {
            for (_,spec) in element.value {
                if let code = spec.generateSwiftCode(context: self) {
                    let file = "\(destination.path)/\(spec.package)/\(spec.short_name)Msg.swift"
//                    let url = URL(fileURLWithPath: file)
//                    try? FileManager.default.createDirectory(at: url.deletingLastPathComponent(), withIntermediateDirectories: true)
                    try? code.write(toFile: file, atomically: false, encoding: .utf8)
                    print("generated \(spec.full_name)")
                }
            }
        }
    }

}

func strip_comments(_ orig: String) -> String {
    return orig.components(separatedBy: "#").first ?? ""
}

func convert_constant_value(field_type: String, val: String) -> Any? {
    switch field_type {
    case "float32", "float64":
        return Float(val)
    case "string":
        return val.trimmingCharacters(in: .whitespaces)
    case "bool":
        return val == "true"
    default:
        return Int(val)
    }
}

func get_msg_file(package: String, base_type: String, search_path: [String: [String]]) -> String? {
    guard search_path.keys.contains(package) else {
        print("Cannot locate message [\(base_type)]: unknown package [\(package)] on search path [\(search_path)]")
        return nil
    }

    for path_tmp in search_path[package]! {
        let path = path_tmp + "/" + base_type + ".msg"
        if FileManager.default.fileExists(atPath: path) {
            return path
        }
    }

    print("Cannot locate message [\(base_type)] in package [\(package)] with paths [\(search_path)]")
    return nil

}
