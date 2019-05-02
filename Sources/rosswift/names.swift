//
//  names.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-09.
//

import Foundation

extension Ros {

    public func getRemappings() -> StringStringMap {
        return globalRemappings
    }

    public func getUnresolvedRemappings() -> StringStringMap {
        return globalUnresolvedRemappings
    }

    internal func initRemap(remappings: StringStringMap) {
        for it in remappings {
            if !it.key.isEmpty && it.key.first! != "_" && it.key != name {
                if let resolvedKey = resolve(name: it.key, remap: false),
                    let resolvedName = resolve(name: it.value, remap: false) {
                    globalRemappings[resolvedKey] = resolvedName
                    globalUnresolvedRemappings[it.key] = it.value
                } else {
                    ROS_ERROR("remapping \(it.key) to \(it.value) failed")
                }
            }
        }
    }

    /// Resolve a graph resource name into a fully qualified graph resource name.
    ///
    /// See http://wiki.ros.org/Names for more details
    ///
    /// - Parameters:
    ///     - name:    Name to resolve
    ///     - remap:   Whether or not to apply remappings to the name
    /// - Returns: nil if the name passed is not a valid graph resource name


    public func resolve(name: String, remap: Bool = true) -> String? {
        return resolve(ns: namespace, name: name, remap: remap)
    }

    /// Resolve a graph resource name into a fully qualified graph resource name.
    ///
    /// See http://wiki.ros.org/Names for more details
    ///
    /// - Parameters:
    ///     - ns:   Namespace to use in resultion
    ///     - name:    Name to resolve
    ///     - remap:   Whether or not to apply remappings to the name
    /// - Throws:
    ///     - invalidName:    if the name passed is not a valid graph resource name



    func resolve(ns: String, name: String, remap: Bool = true) -> String? {
        var error = ""
        guard Names.validate(name: name, error: &error) else {
            ROS_ERROR(error)
            return nil
        }

        if name.isEmpty {
            if ns.isEmpty {
                return "/"
            }

            if ns.first! == "/" {
                return ns
            }

            return Names.append("/", ns)
        }

        var copy = name

        if copy.first! == "~" {
            copy = Names.append(self.name, String(copy.dropFirst()))
        }

        if copy.first! != "/" {
            copy = Names.append("/", Names.append(ns, copy))
        }

        copy = Names.clean(copy)

        if remap {
            return self.remap(name: copy)
        }

        return copy
    }


    /// Apply remappings to a name.
    ///
    /// - Returns: nil if the name is not a valid graph resource name


    public func remap(name: String) -> String? {
        guard let resolved = resolve(name: name, remap: false) else {
            return nil
        }

        if let it = globalRemappings[resolved] {
            return it
        }
        return name
    }

}

/// Contains functions which allow you to manipulate ROS names.

struct Names {

    /// Append one name to another.

    public static func append(_ left: String, _ right: String) -> String {
        return clean(left + "/" + right)
    }

    /// Cleans a graph resource name: removes double slashes, trailing slash.

    public static func clean(_ name: String) -> String {
        let clean = name.replacingOccurrences(of: "//", with: "/")
        if clean.last == "/" {
            return String(clean.dropLast())
        }
        return clean
    }

    public static func isValidCharInName(_ c: Character) -> Bool {
        let a = CharacterSet.alphanumerics
        return  a.contains(c.unicodeScalars.first!) || c == "/" || c == "_"
    }

    enum Errors: Error {
        case invalidName(String)
    }

    /// Get the parent namespace of a name.
    ///
    /// - Parameters:
    ///     - name: The namespace of which to get the parent namespace.
    ///
    /// - Returns: nil if the name is not a valid graph resource name

    public static func parentNamespace(name: String) -> String? {
        var error = ""
        if !validate(name: name, error: &error) {
            ROS_ERROR(error)
            return nil
        }

        if name == "" {
            return ""
        }

        if name == "/" {
            return "/"
        }

        var  strippedName = name

        // rstrip trailing slash
        if name.last == "/" {
            strippedName = String(name.dropLast())
        }

        if let lastPos = strippedName.lastIndex(of: "/") {
            if lastPos == strippedName.startIndex {
                return "/"
            }
            return String(strippedName.prefix(upTo: lastPos))
        }

        return ""
    }


    /// Validate a name against the name spec.

    public static func validate(name: String, error: inout String) -> Bool {
        if name.isEmpty {
            return true
        }
        
        // First element is special, can be only ~ / or alpha
        let c = name.unicodeScalars.first!
        if !CharacterSet.letters.contains(c) && c != "/" && c != "~" {
            error = "Character [\(name.first!)] is not valid as the first" +
                "character in Graph Resource Name [\(name)]. " +
            "Valid characters are a-z, A-Z, / and in some cases ~."
            return false
        }

        for (i, c) in name.dropFirst().enumerated() {
            if !isValidCharInName(c) {
                error = "Character [\(c)] at element [\(i+1)] is not valid " +
                    "in Graph Resource Name [\(name)]. " +
                "Valid characters are a-z, A-Z, 0-9, / and _."

                return false
            }
        }

        return true
    }

    /// Resolve a graph resource name into a fully qualified graph resource name.
    ///
    /// See http://wiki.ros.org/Names for more details
    ///
    /// - Parameters:
    ///     - ns:   Namespace to use in resultion
    ///     - name:    Name to resolve



    static func resolve(ns: String, name: String) -> String? {
        var error = ""
        guard Names.validate(name: name, error: &error) else {
            ROS_ERROR(error)
            return nil
        }

        if name.isEmpty {
            ROS_ERROR(error)
            return nil
        }

        var copy = name

        if copy.first! == "~" {
            ROS_ERROR(error)
            return nil
        }

        if copy.first! != "/" {
            copy = Names.append("/", Names.append(ns, copy))
        }

        copy = Names.clean(copy)

        return copy
    }


}
