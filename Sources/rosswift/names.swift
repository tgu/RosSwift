//
//  names.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-09.
//

import Foundation

extension Ros {

    struct Names {
        private static var globalRemappings = M_string()
        private static var globalUnresolvedRemappings = M_string()

        static func getRemappings() -> M_string {
            return Names.globalRemappings
        }

        static func getUnresolvedRemappings() -> M_string {
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
                error =  "Character [\(name.first!)] is not valid as the first character in Graph Resource Name [\(name)].  Valid characters are a-z, A-Z, / and in some cases ~."
                return false
            }

            for (i,c) in name.enumerated() {
                if !isValidCharInName(c) {
                    error = "Character [\(c)] at element [\(i)] is not valid in Graph Resource Name [\(name)].  Valid characters are a-z, A-Z, 0-9, / and _."

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

        static func remap(_ name: String) -> String {
            let resolved = resolve(name: name, _remap: false)
            if let it = Names.globalRemappings[resolved] {
                return it
            }
            return name
        }

        static func resolve(name: String, _remap: Bool = true) -> String {
            return resolve(ns: this_node.getNamespace(), name: name, _remap: _remap)
        }

        static func resolve(ns: String, name: String, _remap: Bool = true) -> String {
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
                copy = append(this_node.getName(), String(copy.dropFirst()))
            }

            if copy.first! != "/" {
                copy = append("/", append(ns, copy))
            }

            copy = clean(copy)

            if _remap {
                copy = remap(copy)
            }

            return copy
        }

        static func initialize(remappings: M_string) {
            for it in remappings {
                if !it.key.isEmpty && it.key.first! != "_" && it.key != this_node.getName() {
                    let resolved_key = resolve(name: it.key,_remap: false)
                    let resolved_name = resolve(name: it.value,_remap: false)
                    Names.globalRemappings[resolved_key] = resolved_name
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


            var  stripped_name = name

            // rstrip trailing slash
            if name.last == "/" {
                stripped_name = String(name.dropLast())
            }

            #if swift(>=4.2)
            if let last_pos = stripped_name.lastIndex(of: "/") {
                if last_pos == stripped_name.startIndex {
                    return "/"
                }
                return String(stripped_name.prefix(upTo: last_pos))
            }
            #endif
            
            return ""
        }
    }


}
