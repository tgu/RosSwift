//
//  Registrations.swift
//  rosmaster
//
//  Created by Thomas Gustafsson on 2019-04-04.
//

import Foundation
import rpcobject
import rpcclient

struct Caller: Equatable {
    let id: String
    let api: String
}

enum RegistrationType: CaseIterable {
    case topicSubscription
    case topicPublication
    case paramSubscription
    case service
}

/**
 Container for node registration information. Used in master's
self.nodes data structure.  This is effectively a reference
counter for the node registration information: when the
subscriptions and publications are empty the node registration can
be deleted.
*/

struct NodeRef: Equatable {
    let id: String
    let api: String
    var register: [RegistrationType: Set<String>]

    init(node: Caller) {
        self.id = node.id
        self.api = node.api
        self.register = .init()
        for reg in RegistrationType.allCases {
            register[reg] = []
        }
    }

    mutating func clear() {
        for reg in RegistrationType.allCases {
            register[reg] = []
        }
    }

    var isEmpty: Bool {
        return !register.reduce(false, { $0 || !$1.value.isEmpty })
    }

    mutating func add(type: RegistrationType, key: String) {
        register[type]?.insert(key)
    }

    mutating func remove(type: RegistrationType, key: String) {
        register[type]?.remove(key)
    }
}


/**
 All calls may result in access/modifications to node registrations
 dictionary, so be careful to guarantee appropriate thread-safeness.

 Data structure for storing a set of registrations (e.g. publications, services).
 The underlying data storage is the same except for services, which have the
 constraint that only one registration may be active for a given key.
 */

struct State {
    let name: String
    let providers: [String]

    var value: XmlRpcValue {
        return XmlRpcValue(anyArray: [name,providers])
    }
}

extension Result where Success == String, Failure == ErrorMessage {
    var value: XmlRpcValue {
        switch self {
        case .failure(let failure):
            return XmlRpcValue(anyArray: [1, failure.message, 0])
        case .success(let success):
            return XmlRpcValue(anyArray: [1, success, 1])
        }
    }
}

protocol Register {
    var type: RegistrationType { get }
    func unregister(key: String, caller_id: String, api: String) -> Result<String,ErrorMessage>
    func getState() -> [State]

}

class Services: Register {
    let type: RegistrationType = .service
    var services: [String: Caller]
    private let queue = DispatchQueue(label: "Services", attributes: .concurrent)

    init() {
        services = .init()
    }

    func register(key: String, serviceNode: Caller) {
        queue.sync { services[key] = serviceNode }
    }
    
    func unregister(key: String, caller_id: String, api: String) -> Result<String,ErrorMessage> {
        return queue.sync(flags: .barrier) {
            if services[key] != Caller(id: caller_id, api: api) {
                return .success("\(caller_id) is on longer the current service handle for \(key)")
            }
            services.removeValue(forKey: key)
            return .success("Unregistered \(caller_id) as provider of \(key)")
        }
    }


    func getServiceApi(service: String) -> String? {
        return queue.sync { services[service]?.api }
    }

    func registerService(key: String, caller_id: String, caller_api: String, service_api: String) {
        queue.sync(flags: .barrier) {
            services[key] = Caller(id: caller_id, api: service_api)
        }
    }

    func lookupService(key: String) -> Caller? {
        return queue.sync { services[key] }
    }

    func unregisterAll(id: String) {
        queue.sync(flags: .barrier) {
            let remove = services.filter { (key: String, value: Caller) -> Bool in
                value.id == id
            }
            for key in remove.keys {
                services.removeValue(forKey: key)
            }
        }
    }

    func getState() -> [State] {
        var state = [State]()
        for key in services.keys {
            let g = State(name: key, providers: [services[key]!.id] )
            state.append(g)
        }
        return state
    }
}

struct ErrorMessage: Error {
    let message: String
}

class Registrations: Register {
    let type: RegistrationType
    var providers: Multimap<String,Caller>
    private let queue = DispatchQueue(label: "", attributes: .concurrent)

    init(type: RegistrationType) {
        self.type = type
        providers = .init()
    }

    var isEmpty: Bool {
        return queue.sync { providers.isEmpty }
    }

    func getApis(key: String) -> [String] {
        return queue.sync { providers[key].map { $0.api } }
    }

    func getCallers(key: String) -> [Caller] {
        return queue.sync { providers[key] }
    }

    func hasKey(key: String) -> Bool {
        return queue.sync { providers.contains(key: key) }
    }

    func allValues() -> AnySequence<Caller> {
        return queue.sync { providers.values }
    }

    func register(key: String, node: Caller) {
        queue.sync(flags: .barrier) {
            if !self.providers[key].contains(node) {
                self.providers.insert(value: node, forKey: key)
            }
        }
    }
    
    func unregister(key: String, caller_id: String, api: String) -> Result<String,ErrorMessage> {
        let caller = Caller(id: caller_id, api: api)
        return queue.sync(flags: .barrier) {
            if providers[key].contains(caller) {
                providers.removeValue(caller, forKey: key)
                return .success("Unregistered \(caller_id) as provider of \(key)")
            } else {
                return .failure(.init(message: "\(caller_id) is not known provider of \(key)"))
            }
        }
    }


    /// Remove all registrations associated with id

    func unregisterAll(id: String) {
        return queue.sync(flags: .barrier) {
            for key in providers.keys {
                let to_remove = providers[key].filter { $0.id == id }
                for c in to_remove {
                    providers.removeValue(c, forKey: key)
                }
                if providers[key].isEmpty {
                    providers.removeValuesForKey(key)
                }
            }
        }
    }

    func getState() -> [State] {
        var state = [State]()
        queue.sync {
            for key in providers.keys {
                let g = State(name: key, providers: providers[key].map { $0.id })
                state.append(g)
            }
        }
        return state
    }

}


public final class RegistrationManager {
    var paramSubscribers = Registrations(type: .paramSubscription)

    private var nodes: [String: NodeRef] = [:]
    private var publishers = Registrations(type: .topicPublication)
    private var subscribers = Registrations(type: .topicSubscription)
    private var services = Services()
    private let nodeQueue = DispatchQueue(label: "nodes", attributes: .concurrent)


    private func register(type: RegistrationType, key: String, node: Caller) {
        let changed = registerNodeAPI(node: node)

        nodeQueue.sync(flags: .barrier) {
            nodes[node.id]?.add(type: type, key: key)
        }

        if changed {
            publishers.unregisterAll(id: node.id)
            subscribers.unregisterAll(id: node.id)
            paramSubscribers.unregisterAll(id: node.id)
            services.unregisterAll(id: node.id)
        }
    }
    
    private func unregister<T: Register>(r: inout T, key: String, caller_id: String, caller_api: String? = nil, service_api: String? = nil) -> Result<String,ErrorMessage> {

        return nodeQueue.sync(flags: .barrier) {
            if nodes[caller_id] != nil {
                let api = r.type == .service ? service_api! : caller_api!
                let ret = r.unregister(key: key, caller_id: caller_id, api: api)
                if case .success = ret {
                    nodes[caller_id]?.remove(type: r.type, key: key)
                }
                if nodes[caller_id]?.isEmpty ?? false {
                    nodes.removeValue(forKey: caller_id)
                }
                return ret
            } else {
                return .failure(.init(message: "\(caller_id) is not a registered node"))
            }
        }
    }

    private func registerNodeAPI(node: Caller) -> Bool {
        return nodeQueue.sync(flags: .barrier) {
            if let nodeRef = nodes[node.id] {
                if nodeRef.api == node.api {
                    return false
                } else {
                    // FIXME: new node registered with same name
                    // Shutdown old node and unregister
                    let nodeRef = NodeRef(node: node)
                    nodes[node.id] = nodeRef
                    return true
                }

            }

            let nodeRef = NodeRef(node: node)
            nodes[node.id] = nodeRef
            return false
        }
    }

    func publishers(for topic: String) -> [String] {
        return publishers.getApis(key: topic)
    }

    func subscribers(for topic: String) -> [String] {
        return subscribers.getApis(key: topic)
    }

    func state() -> [[XmlRpcValue]] {
        let pub = publishers.getState().map { $0.value }
        let sub = subscribers.getState().map { $0.value }
        let srv = services.getState().map { $0.value }
        return [pub,sub,srv]
    }

    func register(service: String, node: Caller, service_api: String) {
        register(type: .service, key: service, node: node)
        services.register(key: service, serviceNode: Caller(id: node.id, api: service_api) )
    }
    
    func unregister(service: String, caller_id: String, service_api: String) -> Result<String,ErrorMessage> {
        return unregister(r: &services, key: service, caller_id: caller_id, service_api: service_api)
    }
    
    func register(subscriber: Caller, topic: String) {
        register(type: .topicSubscription, key: topic, node: subscriber)
        subscribers.register(key: topic, node: subscriber)
    }
    
    func unregister(subscriber: Caller, topic: String) -> Result<String,ErrorMessage> {
        return unregister(r: &subscribers, key: topic, caller_id: subscriber.id, caller_api: subscriber.api)
    }

    func register(publisher: Caller, topic: String) {
        register(type: .topicPublication, key: topic, node: publisher)
        publishers.register(key: topic, node: publisher)
    }
    
    func unregister(publisher: Caller, topic: String) -> Result<String,ErrorMessage> {
        return unregister(r: &publishers, key: topic, caller_id: publisher.id, caller_api: publisher.api)
    }

    func getNode(caller: String, name: String) -> NodeRef? {
        return nodeQueue.sync {
            return self.nodes[caller]
        }
    }

    func register(parameterSubscriber node: Caller, parameter key: String) {
        let key = resolve(name: key, nameSpace: node.id)
        register(type: .paramSubscription, key: key, node: node)
        paramSubscribers.register(key: key, node: node)
    }

    func unregisterParameterSubscriber(key: String, node: Caller) -> Result<String,ErrorMessage> {
        let key = resolve(name: key, nameSpace: node.id)
        return unregister(r: &paramSubscribers, key: key, caller_id: node.id, caller_api: node.api)
    }

    func lookupService(key: String) -> Caller? {
        return services.lookupService(key: key)
    }




}
