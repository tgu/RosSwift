//
//  param.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation
import rpcobject
import NIO

typealias ParameterStorage = [String: XmlRpcValue]

public typealias SubscribedParameterHandler = @Sendable (XmlRpcValue) -> Void

struct SubscribedParameter: Sendable {
    let name: String
    let handler: SubscribedParameterHandler?
}

/*
 @available(swift 5.1)
 @propertyWrapper
 public struct RosParameter<Value: Sendable>: Sendable {
 public let name: String
 private var cachedValue: Value?
 let rosID: RosID
 var ros: Ros {
 guard let r = Ros.getGlobalRos(for: rosID) else {
 fatalError()
 }
 
 return r
 }
 
 public init(name: String, rosId: RosID) {
 self.name = name
 self.rosID = rosId
 }
 
 public var wrappedValue: Value {
 mutating get {
 _ = ros.param.getSync(name, &cachedValue)
 return cachedValue!
 }
 
 set {
 cachedValue = newValue
 ros.param.setSync(key: name, value: newValue)
 }
 }
 }
 */

/// The node's interface to the ROS parameter server.
///
/// Accessed via `Ros.param`. Provides get/set/delete/search of parameters on
/// the master, plus a local cache (`getCached`) that is kept up to date by
/// subscribing to parameter updates. Isolated as an actor so the cache and
/// subscription state are accessed safely.
public actor Param: Sendable {

    var gSubscribedParameters: [String: SubscribedParameter] = [:]
    var gParameters = ParameterStorage()
    let rosID: RosID

    /// Resolves to the owning Ros; traps with a descriptive message if the
    /// Ros has already been deallocated. All public Param methods require a
    /// live Ros, so this is a contract violation by the caller.
    var ros: Ros {
        guard let r = Ros.getGlobalRos(for: rosID) else {
            fatalError("Param accessed after its owning Ros was deallocated (rosID=\(rosID))")
        }
        return r
    }

    /// Non-trapping accessor for paths that may run after the owning Ros has
    /// been deallocated (e.g. async callbacks scheduled before teardown).
    nonisolated var tryRos: Ros? {
        return Ros.getGlobalRos(for: rosID)
    }

    internal init(rosID: RosID) {
        self.rosID = rosID
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
    
    public func del(key: String) async -> Bool {
        guard let mappedKey = ros.resolve(name: key) else {
            return false
        }
        
        _ = gSubscribedParameters.removeValue(forKey: mappedKey)
        _ = gParameters.removeValue(forKey: mappedKey)
        
        let params = XmlRpcValue(anyArray: [ros.name, mappedKey])
        do {
            let payload = try await ros.master.execute(method: "deleteParam", request: params)
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
    
    
    public func get<T>(_ key: String, _ value: inout T) async -> Bool {
        if let v = await getImpl(key: key, useCache: false) {
            if T.self == XmlRpcValue.self {
                value = v as! T
                return true
            }
            return v.get(val: &value)
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
    
    
    public func get(_ key: String, _ value: inout Int) async -> Bool {
        if let v = await getImpl(key: key, useCache: false) {
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
    
    
    public func getCached<T>(_ key: String, _ value: inout T, completion: SubscribedParameterHandler? = nil) async -> Bool {
        if let v = await getImpl(key: key, useCache: true, completion: completion) {
            if T.self == XmlRpcValue.self {
                value = v as! T
                return true
            }
            return v.get(val: &value)
        }
        return false
    }
    
    /// Gets a parameter using the local cache, subscribing to future updates
    /// so subsequent reads are served locally.
    ///
    /// - Parameters:
    ///   - key: The parameter name.
    ///   - default: The value to return if the parameter is unset or the wrong type.
    ///   - completion: Optional handler invoked when a cached value later changes.
    /// - Returns: The cached/fetched value, or `default`.
    public func getCached<T>(_ key: String, default: T, completion: SubscribedParameterHandler? = nil) async -> T {
        var value = `default`
        if let v = await getImpl(key: key, useCache: true, completion: completion) {
            if T.self == XmlRpcValue.self {
                value = v as! T
                return value
            }
            if v.get(val: &value) {
                return value
            }
        }
        
        // No cache found
        
        await set(key: key, value: value)
        
        return value
    }
    
    
    
    /// Get the list of all the parameters in the server
    ///
    /// - Parameter keys: The vector of all the keys
    /// - Returns: false if the process fails
    
    public func getParamNames(keys: inout [String]) async -> Bool {
        let params = XmlRpcValue(str: ros.name)
        do {
            let parameters = try await ros.master.execute(method: "getParamNames", request: params)
            
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
    
    /// Retrieves the names of all parameters currently on the parameter server.
    /// - Returns: The list of parameter names (empty if the response is malformed).
    /// - Throws: If the master call fails.
    public func getParameterNames() async throws -> [String] {
        let params = XmlRpcValue(str: ros.name)
        let res = try await ros.master.execute(method: "getParamNames", request: params)
        if !res.isArray {
            return []
        }

        var ret = [String]()
        for i in 0..<res.size() {
            guard res[i].isString else {
                return []
            }
            ret.append(res[i].string)
        }
        return ret
    }
    
    /// Check whether a parameter exists on the parameter server.
    ///
    /// - Parameters:
    ///     - key: The key to check
    /// - Returns:
    ///     - true if the parameter exists, false otherwise
    /// - Throws:
    ///     - invalidName    if the key is not a valid graph resource name
    
    public func has(key: String) async -> Bool {
        guard let resolved = ros.resolve(name: key) else {
            return false
        }
        
        let params = XmlRpcValue(anyArray: [ros.name, resolved])
        
        // We don't loop here, because validateXmlrpcResponse() returns false
        // both when we can't contact the master and when the master says, "I
        // don't have that param."
        
        do {
            let payload = try await ros.master.execute(method: "hasParam", request: params)
            
            var result = false
            if payload.get(val: &result) {
                return result
            }
        } catch {
            ROS_ERROR("hasParam failed: \(error)")
        }
        
        return false
    }
    
    
    /// Seeds the parameter server with parameters passed as `_param:=value`
    /// remappings on the command line. Called automatically during node setup.
    /// - Parameter remappings: The node's remapping map.
    public func initialize(remappings: StringStringMap) async {
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
                        await set(key: localName, value: i)
                    } else if let d = Double(param) {
                        await set(key: localName, value: d)
                    } else {
                        switch param {
                        case "true", "True", "TRUE":
                            await set(key: localName, value: true)
                        case "false", "False", "FALSE":
                            await set(key: localName, value: false)
                        default:
                            await set(key: localName, value: param)
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
            if gSubscribedParameters.keys.contains(nsKey) {
                // by erasing the key the parameter will be re-queried
                _ = gParameters.removeValue(forKey: nsKey)
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
    
    public func param<T>(name: String, value: inout T, defaultValue: T) async -> Bool {
        if await has(key: name) {
            if  await get(name, &value) {
                return true
            }
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
    
    public func param<T>(name: String, defaultValue: T) async -> T {
        var value = defaultValue
        _ = await param(name: name, value: &value, defaultValue: defaultValue )
        return value
    }
    
    nonisolated func paramUpdateCallback(params: XmlRpcValue) -> XmlRpcValue {
        let key = params[1].string
        let value = params[2]
        Task { await self.update(key: key, value: value) }
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
    
    
    public func search(key: String, result: inout String) async -> Bool {
        return await search(ns: ros.name, key: key, result: &result)
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
    
    
    public func search(ns: String, key: String, result: inout String) async -> Bool {
        
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
            let payload = try await ros.master.execute(method: "searchParam", request: params)
            result = payload.string
        } catch Master.ValidateError.malformed(let str) {
            ROS_ERROR("error during searchParam \(str)")
            return false
        } catch {
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
    
    
    public func set<T>(key: String, value: T) async {
        guard let mappedKey = ros.resolve(name: key) else {
            ROS_ERROR("Could not set parameter \(key)")
            return
        }
        
        let v = XmlRpcValue(any: value)
        let params = XmlRpcValue(anyArray: [ros.name, mappedKey, v])
        do {
            let parameter = try await ros.master.execute(method: "setParam", request: params)
            if let res = parameter.int {
                if res != 0 {
                    ROS_ERROR("set<T>(key: \(key), value: \(value)) response: \(parameter)")
                }
            } else {
                ROS_ERROR("set<T>(key: \(key), value: \(value)) response: \(parameter)")
            }
            if gSubscribedParameters.keys.contains(mappedKey) {
                gParameters[mappedKey] = v
            }
            invalidateParentParams(mappedKey)
        } catch {
            ROS_ERROR("Could not set parameter \(mappedKey) \(error)")
        }
    }
    
    private func getImpl(key: String, useCache: Bool, completion: SubscribedParameterHandler? = nil) async -> XmlRpcValue? {
        guard var mappedKey = ros.resolve(name: key) else {
            return nil
        }
        
        if mappedKey.isEmpty {
            mappedKey = "/"
        }
        
        var useCache = useCache
        var doReturn = false
        var value: XmlRpcValue?
        
        // Caching only works when ros is started and xmlrpcManager is running
        
        if useCache && ros.isStarted {
            if gSubscribedParameters.keys.contains(mappedKey) {
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
                let params = XmlRpcValue(anyArray: [ros.name,
                                                    ros.xmlrpcManager.serverURI, mappedKey])
                
                do {
                    let result = try await ros.master.execute(method: "subscribeParam", request: params)
                    ROS_DEBUG("cached_parameters: Subscribed to parameter [\(mappedKey)]" +
                              " with result:\n\(result)")
                    gSubscribedParameters[mappedKey] = SubscribedParameter(name: mappedKey, handler: completion)
                } catch {
                    ROS_ERROR("cached_parameters: Subscribe to parameter [\(mappedKey)]:" +
                              " call to the master failed with error \(error)")
                    useCache = false
                }
            }
        }
        
        if doReturn {
            return value
        }
        
        let params = XmlRpcValue(anyArray: [ros.name, mappedKey])
        do {
            let v = try await ros.master.execute(method: "getParam", request: params)
            if v.isArray && v.size() == 1 {
                value = v[0]
            } else {
                value = v
            }
            if useCache {
                ROS_DEBUG("cached_parameters: Caching parameter [\(mappedKey)] with value type [\(String(describing: value))]")
                gParameters[mappedKey] = value
            }
        } catch Master.ValidateError.malformed(let str) {
            ROS_ERROR("\(str)")
            return nil
        } catch let err {
            ROS_DEBUG(err.localizedDescription)
            return nil
        }
        
        return value
    }
    
    
    private func update(key: String, value: XmlRpcValue) {
        let cleanKey = Names.clean(key)
        ROS_DEBUG("cached_parameters: Received parameter update for key [\(cleanKey)] new value: [\(value)]")
        
        if self.gSubscribedParameters.keys.contains(cleanKey) {
            self.gParameters[cleanKey] = value
            self.gSubscribedParameters[cleanKey]?.handler?(value)
        }
        self.invalidateParentParams(cleanKey)
    }
    
    
}

