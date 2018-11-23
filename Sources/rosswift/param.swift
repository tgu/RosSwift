//
//  param.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation

typealias ParameterStorage = [String: XmlRpcValue]

extension Ros {

public struct Param {

    static var parameterQueue = DispatchQueue(label: "parameterQueue")
    static var gSubscribedParameters = Set<String>()
    static var gParameters = ParameterStorage()

    static func set<T>(_ key: String, _ value: T) {
        let mappedKey = Names.resolve(name: key)
        let v = XmlRpcValue(any: value)
        let params = XmlRpcValue(anyArray: [ThisNode.getName(), mappedKey, v])
        do {
            let parameter = try Master.shared.execute(method: "setParam", request: params).wait()
            ROS_DEBUG("set<T> response: \(parameter)")
            if gSubscribedParameters.contains(mappedKey) {
                gParameters[mappedKey] = v
            }
            invalidateParentParams(mappedKey)
        } catch {
            ROS_ERROR("Could not set parameter \(mappedKey) \(error)")
        }
    }

    static func has(key: String) -> Bool {
        let params = XmlRpcValue(anyArray: [ThisNode.getName(), Names.resolve(name: key)])

        // We don't loop here, because validateXmlrpcResponse() returns false
        // both when we can't contact the master and when the master says, "I
        // don't have that param."

        do {
            let payload = try Master.shared.execute(method: "hasParam", request: params).wait()

            var result = false
            if payload.get(val: &result) {
                return result
            }
        } catch {
            ROS_ERROR("hasParam failed: \(error)")
        }

        return false
    }

    static func del(key: String) -> Bool {
        let mappedKey = Names.resolve(name: key)
        parameterQueue.sync {
            gSubscribedParameters.remove(mappedKey)
            gParameters.removeValue(forKey: mappedKey)
        }

        let params = XmlRpcValue(anyArray: [ThisNode.getName(), mappedKey])
        do {
            let payload = try Master.shared.execute(method: "deleteParam", request: params).wait()
            return payload.valid()
        } catch {
            ROS_ERROR("del(key: String) error: \(error)")
        }
        return false
    }

    static func getImpl(key: String, value: inout XmlRpcValue, useCache: Bool) -> Bool {
        var mappedKey = Names.resolve(name: key)
        if mappedKey.isEmpty {
            mappedKey = "/"
        }

        var useCache = useCache
        var doReturn = false
        var ret = true

        if useCache {
            parameterQueue.sync {
                if gSubscribedParameters.contains(mappedKey) {
                    if let it = gParameters[mappedKey] {
                        if it.valid() {
                            ROS_DEBUG("cached_parameters: Using cached parameter value for key [\(mappedKey)]")
                            value = it
                            doReturn = true
                            ret = true
                        } else {
                            ROS_DEBUG("cached_parameters: Cached parameter is invalid for key [\(mappedKey)]")
                            doReturn = true
                            ret = false
                        }
                    }
                } else {
                    if gSubscribedParameters.insert(mappedKey).inserted {
                        let params = XmlRpcValue(anyArray: [ThisNode.getName(),
                                                            XMLRPCManager.instance.serverURI, mappedKey])

                        do {
                            let result = try Master.shared.execute(method: "subscribeParam", request: params).wait()
                            ROS_DEBUG("cached_parameters: Subscribed to parameter [\(mappedKey)]" +
                                " with result:\n\(result)")
                        } catch {
                            ROS_ERROR("cached_parameters: Subscribe to parameter [\(mappedKey)]:" +
                                " call to the master failed with error \(error)")
                            gSubscribedParameters.remove(mappedKey)
                            useCache = false
                        }
                    }
                }
            }
        }

        if doReturn {
            return ret
        }

        let params = XmlRpcValue(anyArray: [ThisNode.getName(), mappedKey])
        do {
            let v = try Master.shared.execute(method: "getParam", request: params).wait()
            if v.isArray && v.size() == 1 {
                value = v[0]
            } else {
                value = v
            }
            if useCache {
                parameterQueue.sync {
                    ROS_DEBUG("cached_parameters: Caching parameter [\(mappedKey)] with value type [\(value.getType())]")
                    gParameters[mappedKey] = value
                }
            }
        } catch {
            ROS_ERROR("\(error)")
            ret = false
        }

        return ret
    }

    public static func get<T>(_ key: String, _ value: inout T) -> Bool {
        var v = XmlRpcValue()
        if getImpl(key: key, value: &v, useCache: false) {
            if T.self == XmlRpcValue.self {
                value = v as! T
                return true
            }
            if v.get(val: &value) {
                return true
            }
        }
        return false
    }

    public static func get(_ key: String, _ value: inout Int) -> Bool {
        var v = XmlRpcValue()
        if getImpl(key: key, value: &v, useCache: false) {
            switch v.value {
            case .int(let i):
                value = i
                return true
            case .double(let d):
                var v = d
                if fmod(d, 1.0) < 0.5 {
                    v = floor(v)
                } else {
                    v = ceil(v)
                }
                if let i = Int(exactly: v) {
                    value = i
                    return true
                }
            default:
                return false
            }
        }
        return false
    }

    public static func getCached<T>(_ key: String, _ value: inout T) -> Bool {
        var v = XmlRpcValue()
        if getImpl(key: key, value: &v, useCache: true) {
            if T.self == XmlRpcValue.self {
                value = v as! T
                return true
            }
            if v.get(val: &value) {
                return true
            }
        }
        return false
    }

    static func invalidateParentParams(_ key: String) {
        var nsKey = Names.parentNamespace(name: key)
        while nsKey != "" && nsKey != "/" {
            if gSubscribedParameters.contains(nsKey) {
                // by erasing the key the parameter will be re-queried
                gParameters.removeValue(forKey: nsKey)
            }
            nsKey = Names.parentNamespace(name: nsKey)
        }
    }

    static func update(key: String, value: XmlRpcValue) {
        let cleanKey = Names.clean(key)
        ROS_DEBUG("cached_parameters: Received parameter update for key [\(cleanKey)]")

        parameterQueue.async {
            if gSubscribedParameters.contains(cleanKey) {
                gParameters[cleanKey] = value
            }
            invalidateParentParams(cleanKey)
        }
    }

    static func paramUpdateCallback(params: XmlRpcValue) -> XmlRpcValue {
        update(key: params[1].string, value: params[2])
        return XmlRpcValue(anyArray: [1, "", 0])
    }

    static func initialize(remappings: StringStringMap) {
        for map in remappings {
            if map.key.count < 2 {
                continue
            }

            let name = map.key
            let param = map.value

            if name.starts(with: "_") && !name.dropFirst().starts(with: "_") {
                let localName = Names.resolve(name: "~".appending(name.dropFirst()))

                if let i = Int32(param) {
                    set(localName, i)
                    continue
                }

                if let d = Double(param) {
                    set(localName, d)
                    continue
                }

                switch param {
                case "true", "True", "TRUE":
                    set(localName, true)
                case "false", "False", "FALSE":
                    set(localName, false)
                default:
                    set(localName, param)
                }

            }
        }

        if !XMLRPCManager.instance.bind(function: "paramUpdate", cb: paramUpdateCallback) {
            ROS_DEBUG("\(#function) Could not bind paramUpdate")
        }
    }

    /// Get the list of all the parameters in the server
    ///
    /// - Parameter keys: The vector of all the keys
    /// - Returns: false if the process fails
    public static func getParamNames(keys: inout [String]) -> Bool {
        let params = XmlRpcValue(str: ThisNode.getName())
        do {
            let parameters = try Master.shared.execute(method: "getParamNames", request: params).wait()

            if !parameters.isArray {
                return false
            }
            keys.removeAll()
            for i in 0..<parameters.size() {
                guard parameters[i].isString else {
                    return false
                }
                keys.append(parameters[i].string)
            }
            return true

        } catch {
            ROS_ERROR("error during getParamNames \(error)")
        }
        return false

    }

    /// Return value from parameter server, or default if unavailable.
    ///
    /// This method tries to retrieve the indicated parameter value from the
    /// parameter server. If the parameter cannot be retrieved, \c default_val
    /// is returned instead.
    ///
    /// - parameter name: The key to be searched on the parameter server.
    /// - parameter defaultValue: Value to return if the server doesn't contain this
    /// parameter.
    ///
    /// - returns: The parameter value retrieved from the parameter server, or
    /// defaultValue if unavailable.

    static func param<T>(name: String, defaultValue: T) -> T {
        var value = defaultValue
        _ = param(name: name, value: &value, defaultValue: defaultValue )
        return value
    }

    /// Assign value from parameter server, with default.
    ///
    /// This method tries to retrieve the indicated parameter value from the
    /// parameter server, storing the result in param_val.  If the value
    /// cannot be retrieved from the server, default_val is used instead.

    /// - Parameter param_name: The key to be searched on the parameter server.
    /// - Parameter param_val: Storage for the retrieved value.
    /// - Parameter default_val: Value to use if the server doesn't contain this
    /// parameter.
    /// - Returns: `true` if the parameter was retrieved from the server, `false` otherwise.

    static func param<T>(name: String, value: inout T, defaultValue: T) -> Bool {
        if has(key: name) {
            if get(name, &value) {
                return true
            }
        }

        value = defaultValue
        return false
    }

    static func search(key: String, result: inout String) -> Bool {
        return search(ns: Ros.ThisNode.getName(), key: key, result: &result)
    }

    static func search(ns: String, key: String, result: inout String) -> Bool {

        // searchParam needs a separate form of remapping -- remapping on the unresolved name, rather than the
        // resolved one.

        var remapped = key
        if let it = Names.getUnresolvedRemappings()[key] {
            remapped = it
        }

        let params = XmlRpcValue(anyArray: [ns, remapped])

        // We don't loop here, because validateXmlrpcResponse() returns false
        // both when we can't contact the master and when the master says, "I
        // don't have that param."

        do {
            let payload = try Master.shared.execute(method: "searchParam", request: params).wait()
            result = payload.string
        } catch {
            ROS_ERROR("error during searchParam \(error)")
            return false
        }

        return true
    }

}

}
