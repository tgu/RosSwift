//
//  names.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-09.
//

import Foundation

extension Ros {

    struct Names {
        private static var globalRemappings = StringStringMap()
        private static var globalUnresolvedRemappings = StringStringMap()

        static func getRemappings() -> StringStringMap {
            return Names.globalRemappings
        }

        static func getUnresolvedRemappings() -> StringStringMap {
            return Names.globalUnresolvedRemappings
        }

        static func isValidCharInName(_ c: Character) -> Bool {
            let a = CharacterSet.alphanumerics
            return  a.contains(c.unicodeScalars.first!) || c == "/" || c == "_"
        }

        static func validate(name: String, error: inout String) -> Bool {
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

            for (i, c) in name.enumerated() {
                if !isValidCharInName(c) {
                    error = "Character [\(c)] at element [\(i)] is not valid " +
                            "in Graph Resource Name [\(name)]. " +
                            "Valid characters are a-z, A-Z, 0-9, / and _."

                    return false
                }
            }

            return true
        }

        static func clean(_ name: String) -> String {
            let clean = name.replacingOccurrences(of: "//", with: "/")
            if clean.last == "/" {
                return String(clean.dropLast())
            }
            return clean
        }

        static func append(_ left: String, _ right: String) -> String {
            return clean(left + "/" + right)
        }

        static func remapName(_ name: String) -> String {
            let resolved = resolve(name: name, remap: false)
            if let it = Names.globalRemappings[resolved] {
                return it
            }
            return name
        }

        static func resolve(name: String, remap: Bool = true) -> String {
            return resolve(ns: ThisNode.getNamespace(), name: name, remap: remap)
        }

        static func resolve(ns: String, name: String, remap: Bool = true) -> String {
            var error = ""
            if !validate(name: name, error: &error) {
                fatalError(error)
            }

            if name.isEmpty {
                if ns.isEmpty {
                    return "/"
                }

                if ns.first! == "/" {
                    return ns
                }

                return append("/", ns)
            }

            var copy = name

            if copy.first! == "~" {
                copy = append(ThisNode.getName(), String(copy.dropFirst()))
            }

            if copy.first! != "/" {
                copy = append("/", append(ns, copy))
            }

            copy = clean(copy)

            if remap {
                copy = remapName(copy)
            }

            return copy
        }

        static func initialize(remappings: StringStringMap) {
            for it in remappings {
                if !it.key.isEmpty && it.key.first! != "_" && it.key != ThisNode.getName() {
                    let resolvedKey = resolve(name: it.key, remap: false)
                    let resolvedName = resolve(name: it.value, remap: false)
                    Names.globalRemappings[resolvedKey] = resolvedName
                    Names.globalUnresolvedRemappings[it.key] = it.value
                }
            }
        }

        static func parentNamespace(name: String) -> String {
            var error = ""
            if !validate(name: name, error: &error) {
                fatalError(error)
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
    }

}
