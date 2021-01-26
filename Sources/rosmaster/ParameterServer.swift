//
//  ParameterServer.swift
//  rosmaster
//
//  Created by Thomas Gustafsson on 2019-04-04.
//

import Foundation
import Logging
import rpcobject

fileprivate let logger = Logger(label: "params")


struct Update<Value: ArrayConstructable> {
    let subscriber: Caller
    let key: String
    let value: Value?

    init(_ subscriber: Caller, _ key: String, _ value: Value? = nil) {
        self.subscriber = subscriber
        self.key = key
        self.value = value
    }
}


final class ParameterServer<Value: ArrayConstructable> {
    typealias ParameterValue = Root<Value>
    let registrationManager: RegistrationManager
    let parameters = RadixTree<Value>()

    init(reg_manager: RegistrationManager) {
        self.registrationManager = reg_manager
    }

    func getAllNames() -> [String] {
        return parameters.getNames()
    }

    func getValueFor(param: String) -> ParameterValue? {
        return parameters.get(param)
    }

    func has(parameter: String) -> Bool {
        return parameters.find(parameter)
    }

    func search(namespace: String, param: String) -> String? {
        if param.isEmpty || isPrivate(param) {
            logger.error("invalid key [\(param) in search_parameter")
            return nil
        }

        if !isGlobal(namespace) {
            logger.error("namespace [\(namespace) must be global in search_parameter")
            return nil
        }

        if isGlobal(param) {
            if has(parameter: param) {
                return param
            } else {
                return nil
            }
        }

        // - we only search for the first namespace in the key to check for a match

        let key_namespaces = param.split(separator: "/")
        let key_ns = String(key_namespaces[0])
        let namespaces = namespace.split(separator: "/")
        for i in 0...namespaces.count {
            let namespace = namespaces.dropLast(i).joined(separator: "/")
            let search_key = "/" + join(namespace: namespace, name: key_ns)
            if has(parameter: search_key) {
                return "/" + join(namespace: namespace, name: param)
            }
        }

        return nil

    }

    func delete(param: String, notfiy: ([Update<Value>]) -> Void) -> Bool {
        if param == "/" || param.isEmpty {
            logger.error("cannot delete root of parameter tree")
            return false
        }

        if let removed = parameters.remove(param) {
            let updates = computeUpdates(key: removed)
            notfiy(updates)
            return true
        }

        return false
    }

    public func set(param: String, value: Value) -> Edge<Value>? {
        if param == "/" && value.count <= 1 {
            logger.error("cannot set root of parameter tree to non-dictionary")
            return nil
        }
        let name = canonicalize(name: param)

        // Split dictionary into children values
        if let dict = value.dictionary {
            for (key, val) in dict {
                let n = join(namespace: name, name: key)
                let _ = set(param: n, value: val)
            }
            return parameters.get(name) as? Edge<Value>
        }

        var par = parameters.get(name) as? Edge<Value>

        if let par = par {
            par.value = value
            par.children.removeAll()
        } else {
            par = parameters.insert(name, value: value)
        }

        guard let parameter = par else {
            logger.error("could not set/create parameter \(name) with value \(value)")
            return nil
        }

        return parameter
    }



    public func set(param: String, value: Value, notfiy: ([Update<Value>]) -> Void) {
        guard let parameter = set(param: param, value: value) else {
            return
        }

        let updates = computeUpdates(key: parameter, param_value: value)
        notfiy(updates)

    }

    func subscribe(parameter: String, node: Caller) -> ParameterValue? {
        let key = canonicalize(name: parameter)
        let val = getValueFor(param: key)
        registrationManager.register(parameterSubscriber: node, parameter: key)
        return val
    }

    func unsubscribe(parameter: String, node: Caller) -> Result<String,ErrorMessage> {
        let key = canonicalize(name: parameter)
        return registrationManager.unregisterParameterSubscriber(key: key, node: node)
    }

    func computeUpdates(key: Edge<Value>, param_value: Value? = nil) -> [Update<Value>] {
        if registrationManager.paramSubscribers.isEmpty {
            return []
        }

        var updates = [Update<Value>]()
        
        var all_keys = [String]()
        key.getNames(&all_keys)
        logger.debug("\(all_keys) has changed")
        for (sub_key, sub_caller) in registrationManager.paramSubscribers.providers {
            let ns_key = sub_key.hasSuffix("/") ? sub_key : sub_key + "/"
            if key.fullName.hasPrefix(ns_key) {
                updates.append(Update<Value>(sub_caller, key.fullName, param_value ))
            } else if ns_key.hasPrefix(key.fullName) && !all_keys.contains(sub_key) {
                // parameter was deleted
                updates.append(Update<Value>(sub_caller, sub_key))
            }
        }

        // add updates for exact matches within tree

        for key in all_keys {
            for s in registrationManager.paramSubscribers.getCallers(key: key) {
                if let par = parameters.get(key) as? Edge<Value> {
                    updates.append(Update(s, key, par.values ))
                } else {
                    logger.error("Could not find parameter \(key)")
                }
            }
        }


        return updates
    }

    func computeAllKeys(param: String, value: XmlRpcValue, all_keys: inout [String] ) {
        guard let vs = value.dictionary else {
            return
        }

        for (k, value) in vs {
            let newKey = join(namespace: param, name: k) + "/"
            all_keys.append(newKey)
            if let _ = value.dictionary {
                computeAllKeys(param: newKey, value: value, all_keys: &all_keys)
            }
        }
    }


}

