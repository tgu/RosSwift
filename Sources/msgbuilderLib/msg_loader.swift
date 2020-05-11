//
//  msg_loader.swift
//  msgbuilder
//
//  Created by Thomas Gustafsson on 2019-04-29.
//

import Foundation
import StdMsgs
import msgs

protocol BaseMsg {
    var package: String { get }
    var short_name: String { get }
    var full_name: String { get }
    var generate: Bool { get }
    func dumpSwiftCode(context: MsgContext, destination: String)
}

extension BaseMsg {
}

public final class MsgContext {
    var registered: [String: [String: BaseMsg] ]
    var files: [String: String]
    var dependencies: [String: [MsgSpec]]
    var embed: Bool

    public init(useBuiltin: Bool, embed: Bool = true) {
        registered = [:]
        files = [:]
        dependencies = [:]
        self.embed = embed

        // used in common_msgs and needed for md5 computation
        
        register(message: std_msgs.Header.self, serviceMsg: false, generate: false)
        register(message: std_msgs.ColorRGBA.self, serviceMsg: false, generate: false)

        if useBuiltin {
            for (_, value) in geometry_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }

            for (_, value) in uuid_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }

            for (_, value) in geographic_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }

            for (_, value) in actionlib_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }

            for (_, value) in control_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }

            for (_, value) in map_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }


            for (_, value) in nav_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }

            for (_, value) in pcl_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }

            for (_, value) in rosgraph_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }

            for (_, value) in sensor_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }

            for (_, value) in shape_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }

            for (_, value) in stereo_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }

            for (_, value) in trajectory_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }

            for (_, value) in visualization_msgs.all.enumerated() {
                register(message: value.value, serviceMsg: false, generate: false)
            }


        }


    }

    func register(message: Message.Type, serviceMsg: Bool, generate: Bool = true) {
        _ = addMsg(with: message.definition, full_name: message.datatype, serviceMsg: serviceMsg, generate: generate)
    }

    func register(full_msg_type: String, msgspec: BaseMsg) {
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
        return registered[package]?[base_type] as? MsgSpec
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

    public func load_dir(path: URL, package_name: String) {
        print("===== \(path.path)")
        guard let content = try? FileManager.default.contentsOfDirectory(atPath: path.path) else {
            print("no files in directory \(path)")
            exit(1)
        }

        let packages = content.filter { $0.hasSuffix("_msgs") || $0.hasSuffix("_pkgs")}
        let files = content.filter { $0.hasSuffix(".msg") || $0.hasSuffix(".srv") }

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
        } else {
            if content.contains("msg") {
                let sub_path = path.appendingPathComponent("msg")
                load_dir(path: sub_path, package_name: package_name)
            }
            if content.contains("srv") {
                let sub_path = path.appendingPathComponent("srv")
                load_dir(path: sub_path, package_name: package_name)
            }
        }
    }

    public func addMsg(with content: String, full_name: String, serviceMsg: Bool, generate: Bool) -> MsgSpec? {
        if let spec = MsgSpec(text: content, full_name: full_name, serviceMessage: serviceMsg, generate: generate) {
            register(full_msg_type: full_name, msgspec: spec)
            return spec
        } else {
            print("\(full_name) = no spec")
            return nil
        }
    }

    func addSrv(with content: String, full_name: String) -> SrvSpec? {
        let parts = content.components(separatedBy: "---\n")
        guard parts.count == 2 else {
            return nil
        }
        guard let msg_in = MsgSpec(text: parts[0], full_name: full_name+"Request", serviceMessage: true, generate: true) else {
            return nil
        }
        guard let msg_out = MsgSpec(text: parts[1], full_name: full_name+"Response", serviceMessage: true, generate: true) else {
            return nil
        }
        if let (package, shortName) = package_resource_name(name: full_name) {
            register(full_msg_type: msg_in.full_name, msgspec: msg_in)
            register(full_msg_type: msg_out.full_name, msgspec: msg_out)
            let srv =  SrvSpec(request: msg_in, response: msg_out, text: content, full_name: full_name, package: package, short_name: shortName, generate: true)
            register(full_msg_type: full_name, msgspec: srv)
            return srv
        }

        return nil
    }

    func loadMsg(from path: String, full_name: String) -> BaseMsg? {
        if let content = try? String(contentsOfFile: path) {
            if path.hasSuffix("msg"), let spec = addMsg(with: content, full_name: full_name, serviceMsg: false, generate: true) {
                set_file(full_msg_type: full_name, file_path: path)
                return spec
            } else if path.hasSuffix("srv"), let srv = addSrv(with: content, full_name: full_name) {
                return srv
            }
        } else {
            print("Could not open \(path)")
        }
        return nil
    }

    func load_srv_from_file(from path: String, full_name: String, package: String, shortName: String) -> SrvSpec? {
        if let content = try? String(contentsOfFile: path) {
            let parts = content.components(separatedBy: "---\n")
            guard parts.count == 2 else {
                return nil
            }
            guard let msg_in = MsgSpec(text: parts[0], full_name: full_name+"Request", serviceMessage: true, generate: true) else {
                return nil
            }
            guard let msg_out = MsgSpec(text: parts[1], full_name: full_name+"Response", serviceMessage: true, generate: true) else {
                return nil
            }
            return SrvSpec(request: msg_in, response: msg_out, text: content, full_name: full_name, package: package, short_name: shortName, generate: true)
        } else {
            print("Could not open \(path)")
        }
        return nil
    }



    func load_msg_by_type(msg_type: String, search_path: [String: [String]]) -> BaseMsg? {
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

    func load_srv_by_type(srv_type: String, search_path: [String: [String]]) -> SrvSpec? {
        if let (package, base_type) = package_resource_name(name: srv_type) {
            if let file_path = get_srv_file(package: package, base_type: base_type, search_path: search_path) {
                let spec = load_srv_from_file(from: file_path, full_name: srv_type, package: package, shortName: base_type)
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
                depspec = get_registered(msg_type: resolved_type)
            } else {
                depspec = load_msg_by_type(msg_type: resolved_type, search_path: search_path) as? MsgSpec
            }

            if let depspec = depspec {
                depends.append(depspec)
                if !dependencies.keys.contains(resolved_type) {
                    _ = load_msg_depends(spec: depspec, search_path: search_path)
                }
            } else {
                print("Could not load resolved_type: \(resolved_type)")
            }

        }
        set_depends(full_msg_type: spec.full_name, dependencies: depends)
        return depends
    }

    public func genAllMessages(to destination: URL) {
        for package in registered.keys {
            let mess = registered[package]!
            .values.compactMap { $0 as? MsgSpec }
            if !mess.contains(where: { $0.generate }) {
                continue
            }

            var messages = mess
                .filter { !$0.serviceMessage }
                .map { "\"\($0.short_name)\": \($0.short_name).self"}
                .joined(separator: ",\n\t\t")

            if messages.isEmpty {
                messages = ":"
            }

            let code = """
            // Generated by msgbuilder \(Date())

            import StdMsgs

            public enum \(package) {
                public static let all: [String: Message.Type] = [
                    \(messages)]
            }
            """

            let file = "\(destination.path)/\(package)/\(package).swift"
            if let oldContent = try? String(contentsOfFile: file, encoding: .utf8) {
                // The date in the first row will always change
                if oldContent.components(separatedBy: .newlines).dropFirst() == code.components(separatedBy: .newlines).dropFirst() {
                    continue
                }
            }
            let url = URL(fileURLWithPath: file)
            try? FileManager.default.createDirectory(at: url.deletingLastPathComponent(), withIntermediateDirectories: true)
            if embed {
                try? code.write(toFile: file, atomically: false, encoding: .utf8)
            }
        }

        for (_,element) in registered.enumerated() {
            for (_,spec) in element.value.filter({$0.value.generate}) {
                spec.dumpSwiftCode(context: self, destination: destination.path)
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

func get_srv_file(package: String, base_type: String, search_path: [String: [String]]) -> String? {
    return get_msg_file(package: package, base_type: base_type, search_path: search_path, ext: "srv")
}

func get_msg_file(package: String, base_type: String, search_path: [String: [String]], ext: String = "msg") -> String? {
    guard search_path.keys.contains(package) else {
        print("Cannot locate message [\(base_type)]: unknown package [\(package)] on search path [\(search_path)]")
        return nil
    }

    for path_tmp in search_path[package]! {
        let path = "\(path_tmp)/\(base_type).\(ext)"
        if FileManager.default.fileExists(atPath: path) {
            return path
        }
    }

    print("Cannot locate message [\(base_type)] in package [\(package)] with paths [\(search_path)]")
    return nil

}
