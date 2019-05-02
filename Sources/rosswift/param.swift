//
//  param.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation

typealias ParameterStorage = [String: XmlRpcValue]

public final class Param {

    let parameterQueue = DispatchQueue(label: "parameterQueue")
    var gSubscribedParameters = Set<String>()
    var gParameters = ParameterStorage()
    unowned var ros: Ros!

    internal init() {
    }

    /// Delete a parameter from the parameter server.
    ///
    /// - Parameters:
    ///     - key:    The key to delete.
    /// - Returns:
    ///    - true: if the deletion succeeded
    ///    - false: otherwise.
    /// - Throws:
    ///     - invalidName    if the key is not a valid graph resource name

    public func del(key: String) -> Bool {
        guard let mappedKey = ros.resolve(name: key) else {
            return false
        }

        parameterQueue.sync {
            gSubscribedParameters.remove(mappedKey)
            gParameters.removeValue(forKey: mappedKey)
        }

        let params = XmlRpcValue(anyArray: [ros.name, mappedKey])
        do {
            let payload = try ros.master.execute(method: "deleteParam", request: params).wait()
            return payload.valid()
        } catch {
            ROS_ERROR("del(key: String) error: \(error)")
        }
        return false
    }

    /// Get a value from the parameter server.
    ///
    /// - Parameters:
    ///     - key:    The key to be used in the parameter server's dictionary.
    ///     - value:  Storage for the retrieved value
    /// - Returns:
    ///    - true: if the parameter value was retrieved
    ///    - false: otherwise.
    /// - Throws:
    ///     - invalidName    if the key is not a valid graph resource name


    public func get<T>(_ key: String, _ value: inout T) -> Bool {
        if let v = getImpl(key: key, useCache: false) {
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

    /// Get an integer value from the parameter server.
    ///
    /// - Parameters:
    ///     - key:    The key to be used in the parameter server's dictionary.
    ///     - value:  Storage for the retrieved value
    /// - Returns:
    ///    - true: if the parameter value was retrieved
    ///    - false: otherwise.
    /// - Throws:
    ///     - invalidName    if the key is not a valid graph resource name


    public func get(_ key: String, _ value: inout Int) -> Bool {
        if let v = getImpl(key: key, useCache: false) {
            switch v {
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

    /// Get a value from the parameter server, with local caching.
    ///
    /// This function will cache parameters locally, and subscribe for updates from the
    /// parameter server. Once the parameter is retrieved for the first time no subsequent
    /// getCached() calls with the same key will query the master – they will instead look up
    /// in the local cache.
    ///
    /// - Parameters:
    ///     - key:    The key to be used in the parameter server's dictionary.
    ///     - value:  Storage for the retrieved value
    /// - Returns:
    ///    - true: if the parameter value was retrieved
    ///    - false: otherwise.
    /// - Throws:
    ///     - invalidName    if the key is not a valid graph resource name


    public func getCached<T>(_ key: String, _ value: inout T) -> Bool {
        if let v = getImpl(key: key, useCache: true) {
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

    /// Get the list of all the parameters in the server
    ///
    /// - Parameter keys: The vector of all the keys
    /// - Returns: false if the process fails

    public func getParamNames(keys: inout [String]) -> Bool {
        let params = XmlRpcValue(str: ros.name)
        do {
            let parameters = try ros.master.execute(method: "getParamNames", request: params).wait()

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

    /// Check whether a parameter exists on the parameter server.
    ///
    /// - Parameters:
    ///     - key: The key to check
    /// - Returns:
    ///     - true if the parameter exists, false otherwise
    /// - Throws:
    ///     - invalidName    if the key is not a valid graph resource name

    public func has(key: String) -> Bool {
        guard let resolved = ros.resolve(name: key) else {
            return false
        }

        let params = XmlRpcValue(anyArray: [ros.name, resolved])

        // We don't loop here, because validateXmlrpcResponse() returns false
        // both when we can't contact the master and when the master says, "I
        // don't have that param."

        do {
            let payload = try ros.master.execute(method: "hasParam", request: params).wait()

            var result = false
            if payload.get(val: &result) {
                return result
            }
        } catch {
            ROS_ERROR("hasParam failed: \(error)")
        }

        return false
    }


    public func initialize(remappings: StringStringMap) {
        for map in remappings {

            let name = map.key
            let param = map.value

            // Don't allow short names

            if name.count < 2 {
                ROS_ERROR("Will not remap \(name) to \(param)")
                continue
            }

            if name.starts(with: "_") && !name.dropFirst().starts(with: "_") {
                if let localName = ros.resolve(name: "~".appending(name.dropFirst())) {
                    if let i = Int32(param) {
                        set(key: localName, value: i)
                    } else if let d = Double(param) {
                        set(key: localName, value: d)
                    } else {
                        switch param {
                        case "true", "True", "TRUE":
                            set(key: localName, value: true)
                        case "false", "False", "FALSE":
                            set(key: localName, value: false)
                        default:
                            set(key: localName, value: param)
                        }
                    }
                }
            }
        }

        if !ros.xmlrpcManager.bind(function: "paramUpdate", cb: paramUpdateCallback) {
            ROS_DEBUG("\(#function) Could not bind paramUpdate")
        }
    }

    func invalidateParentParams(_ key: String) {
        guard var nsKey = Names.parentNamespace(name: key) else {
            return
        }

        while nsKey != "" && nsKey != "/" {
            if gSubscribedParameters.contains(nsKey) {
                // by erasing the key the parameter will be re-queried
                gParameters.removeValue(forKey: nsKey)
            }

            // This should always succeed
            nsKey = Names.parentNamespace(name: nsKey)!
        }
    }

    /// Assign value from parameter server, with default.
    ///
    /// This method tries to retrieve the indicated parameter value from the
    /// parameter server, storing the result in `value`.  If the value
    /// cannot be retrieved from the server, `defaultValue` is used instead.

    /// - Parameters:
    ///     - name: The key to be searched on the parameter server.
    ///     - value: Storage for the retrieved value.
    ///     - defaultValue: Value to use if the server doesn't contain this parameter.
    /// - Returns: `true` if the parameter was retrieved from the server, `false` otherwise.

    public func param<T>(name: String, value: inout T, defaultValue: T) -> Bool {
        if has(key: name) && get(name, &value) {
            return true
        }

        value = defaultValue
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

    public func param<T>(name: String, defaultValue: T) -> T {
        var value = defaultValue
        _ = param(name: name, value: &value, defaultValue: defaultValue )
        return value
    }

    func paramUpdateCallback(params: XmlRpcValue) -> XmlRpcValue {
        update(key: params[1].string, value: params[2])
        return XmlRpcValue(anyArray: [1, "", 0])
    }

    /// Search up the tree for a parameter with a given key. This version defaults to starting in the current node's name.
    /// This function parameter server's searchParam feature to search up the tree for a parameter. For
    /// example, if the parameter server has a parameter [/a/b] and you specify the namespace [/a/c/d],
    /// searching for the parameter "b" will yield [/a/b]. If [/a/c/d/b] existed, that parameter would be
    /// returned instead
    ///
    /// - Parameters:
    ///     - key: parameter to search for
    ///     - result: the found value (if any)
    ///
    /// - Returns: true if the parameter was found, false otherwise.


    public func search(key: String, result: inout String) -> Bool {
        return search(ns: ros.name, key: key, result: &result)
    }


    /// Search up the tree for a parameter with a given key.
    ///
    /// This function parameter server's searchParam feature to search up the tree for a parameter. For
    /// example, if the parameter server has a parameter [/a/b] and you specify the namespace [/a/c/d],
    /// searching for the parameter "b" will yield [/a/b]. If [/a/c/d/b] existed, that parameter would be
    /// returned instead.
    ///
    /// - Parameters:
    ///     - ns: The namespace to begin the search in
    ///     - key: parameter to search for
    ///     - result: the found value (if any)
    ///
    /// - Returns: true if the parameter was found, false otherwise.


    public func search(ns: String, key: String, result: inout String) -> Bool {

        // searchParam needs a separate form of remapping -- remapping on the unresolved name, rather than the
        // resolved one.

        var remapped = key
        if let it = ros.getUnresolvedRemappings()[key] {
            remapped = it
        }

        let params = XmlRpcValue(anyArray: [ns, remapped])

        // We don't loop here, because validateXmlrpcResponse() returns false
        // both when we can't contact the master and when the master says, "I
        // don't have that param."

        do {
            let payload = try ros.master.execute(method: "searchParam", request: params).wait()
            result = payload.string
        } catch {
            ROS_ERROR("error during searchParam \(error)")
            return false
        }

        return true
    }


    /// Set an arbitrary value of type T on the parameter server
    ///
    /// - Parameters:
    ///     - key:    The key to be used in the parameter server's dictionary.
    ///     - value:  The value to be inserted
    /// - Throws:
    ///     - invalidName    if the key is not a valid graph resource name


    public func set<T>(key: String, value: T) {
        guard let mappedKey = ros.resolve(name: key) else {
            ROS_ERROR("Could not set parameter \(key)")
            return
        }

        let v = XmlRpcValue(any: value)
        let params = XmlRpcValue(anyArray: [ros.name, mappedKey, v])
        do {
            let parameter = try ros.master.execute(method: "setParam", request: params).wait()
            ROS_DEBUG("set<T> response: \(parameter)")
            if gSubscribedParameters.contains(mappedKey) {
                gParameters[mappedKey] = v
            }
            invalidateParentParams(mappedKey)
        } catch {
            ROS_ERROR("Could not set parameter \(mappedKey) \(error)")
        }
    }

    private func getImpl(key: String, useCache: Bool) -> XmlRpcValue? {
        guard var mappedKey = ros.resolve(name: key) else {
            return nil
        }

        if mappedKey.isEmpty {
            mappedKey = "/"
        }

        var useCache = useCache
        var doReturn = false
        var value: XmlRpcValue?

        if useCache {
            parameterQueue.sync {
                if gSubscribedParameters.contains(mappedKey) {
                    if let it = gParameters[mappedKey] {
                        doReturn = true
                        if it.valid() {
                            ROS_DEBUG("cached_parameters: Using cached parameter value for key [\(mappedKey)]")
                            value = it
                        } else {
                            ROS_DEBUG("cached_parameters: Cached parameter is invalid for key [\(mappedKey)]")
                        }
                    }
                } else {
                    if gSubscribedParameters.insert(mappedKey).inserted {
                        let params = XmlRpcValue(anyArray: [ros.name,
                                                            ros.xmlrpcManager.serverURI, mappedKey])

                        do {
                            let result = try ros.master.execute(method: "subscribeParam", request: params).wait()
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
            return value
        }

        let params = XmlRpcValue(anyArray: [ros.name, mappedKey])
        do {
            let v = try ros.master.execute(method: "getParam", request: params).wait()
            if v.isArray && v.size() == 1 {
                value = v[0]
            } else {
                value = v
            }
            if useCache {
                parameterQueue.sync {
                    ROS_DEBUG("cached_parameters: Caching parameter [\(mappedKey)] with value type [\(String(describing: value))]")
                    gParameters[mappedKey] = value
                }
            }
        } catch {
            ROS_ERROR("\(error)")
            return nil
        }

        return value
    }


    private func update(key: String, value: XmlRpcValue) {
        let cleanKey = Names.clean(key)
        ROS_DEBUG("cached_parameters: Received parameter update for key [\(cleanKey)] new value: [\(value)]")

        parameterQueue.async {
            if self.gSubscribedParameters.contains(cleanKey) {
                self.gParameters[cleanKey] = value
            }
            self.invalidateParentParams(cleanKey)
        }
    }


}

