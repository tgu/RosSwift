//
//  param.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation

extension Ros {

public struct param {

    typealias M_Param = [String:XmlRpcValue]

    static var g_params_mutex = DispatchQueue(label: "g_params_mutex")
    static var g_subscribed_params = Set<String>()
    static var g_params = M_Param()

    static func set<T>(_ key: String, _ value: T) {
        let mapped_key = Names.resolve(name: key)
        let v = XmlRpcValue(any: value)
        let params = XmlRpcValue(anyArray: [this_node.getName(),mapped_key,v])
        do {
            let p = try Master.shared.execute(method: "setParam", request: params).wait()
            ROS_DEBUG("set<T> response: \(p)")
            if g_subscribed_params.contains(mapped_key) {
                g_params[mapped_key] = v
            }
            invalidateParentParams(mapped_key)
        } catch {
            ROS_ERROR("Could not set parameter \(mapped_key) \(error)")
        }

//        g_params_mutex.sync {
//            var result = XmlRpcValue()
//            var payload = XmlRpcValue()
//            if Master.shared.execute(method: "setParam", request: params, response: &result, payload: &payload, wait_for_master: true) {
//                if g_subscribed_params.contains(mapped_key) {
//                    g_params[mapped_key] = v
//                }
//                invalidateParentParams(mapped_key)
//            }
//        }
    }

    static func has(key: String) -> Bool {
//        var result = XmlRpcValue()
//        var payload = XmlRpcValue()
        let params = XmlRpcValue(anyArray: [this_node.getName(),Names.resolve(name: key)])


        // We don't loop here, because validateXmlrpcResponse() returns false
        // both when we can't contact the master and when the master says, "I
        // don't have that param."

        do {
            let payload = try! Master.shared.execute(method: "hasParam", request: params).wait()

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
        let mapped_key = Names.resolve(name: key)
        g_params_mutex.sync {
            g_subscribed_params.remove(mapped_key)
            g_params.removeValue(forKey: mapped_key)
        }

//        var result = XmlRpcValue()
//        var payload = XmlRpcValue()
        let params = XmlRpcValue(anyArray: [this_node.getName(),mapped_key])
        do {
            let payload = try Master.shared.execute(method: "deleteParam", request: params).wait()
            return payload.valid()
        } catch {
            ROS_ERROR("del(key: String) error: \(error)")
        }
//        if !Master.shared.execute(method: "deleteParam", request: params, response: &result, payload: &payload, wait_for_master: false) {
//            return false
//        }

        return false
    }

    static func getImpl(key: String, value: inout XmlRpcValue, use_cache: Bool) -> Bool {
        var mapped_key = Names.resolve(name: key)
        if mapped_key.isEmpty {
            mapped_key = "/"
        }

        var useCache = use_cache
        var doReturn = false
        var ret = true

        if useCache {
            g_params_mutex.sync {
                if g_subscribed_params.contains(mapped_key) {
                    if let it = g_params[mapped_key] {
                        if it.valid() {
                            ROS_DEBUG("cached_parameters: Using cached parameter value for key [\(mapped_key)]")
                            value = it
                            doReturn = true
                            ret = true
                        } else {
                            ROS_DEBUG("cached_parameters: Cached parameter is invalid for key [\(mapped_key)]")
                            doReturn = true
                            ret = false
                        }
                    }
                } else {
                    if g_subscribed_params.insert(mapped_key).inserted {
//                        var result = XmlRpcValue()
//                        var payload = XmlRpcValue()
                        let params = XmlRpcValue(anyArray: [this_node.getName(),XMLRPCManager.instance.serverURI,mapped_key])

                        do {
                            let result = try Master.shared.execute(method: "subscribeParam", request: params).wait()
                            ROS_DEBUG("cached_parameters: Subscribed to parameter [\(mapped_key)] with result:\n\(result)")
                        } catch {
                            ROS_ERROR("cached_parameters: Subscribe to parameter [\(mapped_key)]: call to the master failed with error \(error)")
                            g_subscribed_params.remove(mapped_key)
                            useCache = false
                        }

//                        if !Master.shared.execute(method: "subscribeParam", request: params, response: &result, payload: &payload, wait_for_master: false) {
//                            ROS_DEBUG("cached_parameters", "Subscribe to parameter [\(mapped_key)]: call to the master failed")
//                            g_subscribed_params.remove(mapped_key)
//                            useCache = false
//                        } else {
//                            ROS_DEBUG("cached_parameters", "Subscribed to parameter [\(mapped_key)]")
//                        }
                    }
                }
            }
        }

        if doReturn {
            return ret
        }

//        var result = XmlRpcValue()
        let params = XmlRpcValue(anyArray: [this_node.getName(),mapped_key])
        do {
            let v = try Master.shared.execute(method: "getParam", request: params).wait()
            if v.isArray && v.size() == 1 {
                value = v[0]
            } else {
                value = v
            }
            if useCache {
                g_params_mutex.sync {
                    ROS_DEBUG("cached_parameters: Caching parameter [\(mapped_key)] with value type [\(value.getType())]")
                    g_params[mapped_key] = value
                }
            }
        } catch {
            ROS_ERROR("\(error)")
            ret = false
        }
//        guard let v = Master.shared.execute(method: "getParam", request: params) else  {
//            return false
//        }
//        ret = Master.shared.execute(method: "getParam", request: params, response: &result, payload: &value, wait_for_master: false)
//        value = v[0]
//        if useCache {
//            g_params_mutex.sync {
//                ROS_DEBUG("cached_parameters", "Caching parameter [\(mapped_key)] with value type [\(value.getType())]")
//                g_params[mapped_key] = value
//            }
//        }

        return ret
    }

    public static func get<T>(_ key: String, _ value: inout T) -> Bool {
        var v = XmlRpcValue()
        if getImpl(key: key, value: &v, use_cache: false) {
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
        if getImpl(key: key, value: &v, use_cache: false) {
            switch v.value {
            case .int(let i):
                value = i
                return true
            case .double(let d):
                var v = d
                if fmod(d,1.0) < 0.5 {
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
        if getImpl(key: key, value: &v, use_cache: true) {
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
        var ns_key = Names.parentNamespace(name: key)
        while ns_key != "" && ns_key != "/" {
            if g_subscribed_params.contains(ns_key) {
                // by erasing the key the parameter will be re-queried
                g_params.removeValue(forKey: ns_key)
            }
            ns_key = Names.parentNamespace(name: ns_key)
        }
    }

    static func update(key: String, value: XmlRpcValue) {
        let clean_key = Names.clean(key)
        ROS_DEBUG("cached_parameters: Received parameter update for key [\(clean_key)]")

        g_params_mutex.async {
            if g_subscribed_params.contains(clean_key) {
                g_params[clean_key] = value
            }
            invalidateParentParams(clean_key)
        }
    }


    static func paramUpdateCallback(params: XmlRpcValue) -> XmlRpcValue
    {
        update(key: params[1].string, value: params[2])
        return XmlRpcValue(anyArray: [1,"",0])
    }


    static func initialize(remappings: M_string) {
        for map in remappings {
            if map.key.count < 2 {
                continue
            }

            let name = map.key
            let param = map.value

            if name.starts(with: "_") && !name.dropFirst().starts(with: "_") {
                let local_name = Names.resolve(name: "~".appending(name.dropFirst()))

                if let i = Int32(param) {
                    set(local_name,i)
                    continue
                }

                if let d = Double(param) {
                    set(local_name,d)
                    continue
                }

                switch param {
                case "true","True","TRUE":
                    set(local_name,true)
                case "false","False","FALSE":
                    set(local_name,false)
                default:
                    set(local_name,param)
                }

            }
        }

        if !XMLRPCManager.instance.bind(function_name: "paramUpdate", cb: paramUpdateCallback) {
            ROS_DEBUG("\(#function) Could not bind paramUpdate")
        }
    }

    /// Get the list of all the parameters in the server
    ///
    /// - Parameter keys: The vector of all the keys
    /// - Returns: false if the process fails
    public static func getParamNames(keys: inout [String]) -> Bool {
        let params = XmlRpcValue(str: this_node.getName())
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


//        if result.size() != 3 {
//            ROS_DEBUG("\(#function) result has \(result.size()) values, should be 3 trying child")
//            result = result[0]
//            if result.size() != 3 {
//                ROS_DEBUG("\(#function) chile has not three values one")
//                return false
//            }
//        }
//
//        let parameters = result[2]
    }


    /**
     * \brief Return value from parameter server, or default if unavailable.
     *
     * This method tries to retrieve the indicated parameter value from the
     * parameter server. If the parameter cannot be retrieved, \c default_val
     * is returned instead.
     *
     * \param param_name The key to be searched on the parameter server.
     *
     * \param default_val Value to return if the server doesn't contain this
     * parameter.
     *
     * \return The parameter value retrieved from the parameter server, or
     * \c default_val if unavailable.
     *
     * \throws InvalidNameException If the key is not a valid graph resource name.
     */

    static func param<T>(param_name: String, default_val: T) -> T
    {
        var value = default_val
        param(param_name: param_name, param_val: &value, default_val: default_val )
        return value
    }


    /** Assign value from parameter server, with default.

      This method tries to retrieve the indicated parameter value from the
      parameter server, storing the result in param_val.  If the value
      cannot be retrieved from the server, default_val is used instead.

     - Parameter param_name: The key to be searched on the parameter server.
     - Parameter param_val: Storage for the retrieved value.
     - Parameter default_val: Value to use if the server doesn't contain this
      parameter.
    - Returns `true` if the parameter was retrieved from the server, `false` otherwise.
     */

    static func param<T>(param_name: String, param_val: inout T,  default_val: T) -> Bool
    {
        if has(key: param_name) {
            if get(param_name, &param_val)  {
                return true
            }
        }

        param_val = default_val
        return false
    }


    static func search(key: String, result: inout String) -> Bool {
        return search(ns: Ros.this_node.getName(), key: key, result: &result)
    }

    static func search(ns: String, key: String, result: inout String) -> Bool {

        // searchParam needs a separate form of remapping -- remapping on the unresolved name, rather than the
        // resolved one.

        var remapped = key
        if let it = Names.getUnresolvedRemappings()[key]  {
            remapped = it
        }

        let params = XmlRpcValue(anyArray: [ns,remapped])

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
