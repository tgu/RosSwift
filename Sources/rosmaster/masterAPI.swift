//
//  masterAPI.swift
//  rosmaster
//
//  Created by Thomas Gustafsson on 2019-04-04.
//

import Foundation
import Logging
import rpcclient
import rpcobject

fileprivate let logger = Logger(label: "masterapi")

enum CommunicationProtocol: String {
    case http
    case rosrpc
}

struct API: CustomStringConvertible {
    let `protocol`: CommunicationProtocol
    let host: String
    let port: UInt16

    init?(api: String) {
        var str = api.dropFirst(0)
        if api.hasPrefix("http://") {
            str = str.dropFirst(7)
            `protocol` = .http
        } else if api.hasPrefix("rosrpc://") {
            str = str.dropFirst(9)
            `protocol` = .rosrpc
        } else {
            return nil
        }
        if str.hasSuffix("/") {
            str = str.dropLast()
        }
        let parts = str.split(separator: ":")
        guard parts.count == 2 else {
            return nil
        }

        host = String(parts[0])
        guard let p = UInt16(parts[1]) else {
            return nil
        }
        port = p
    }

    var description: String {
        return "\(`protocol`.rawValue)://\(host):\(port)"
    }

}

final class RosMasterHandler {
    let parameterServer: ParameterServer<XmlRpcValue>
    let rm: RegistrationManager
    var topics: [String: String]

    init() {
        rm = .init()
        topics = .init()
        parameterServer = .init(reg_manager: rm)
    }

    func executeMethod(methodName: String, params: [XmlRpcValue]) -> XmlRpcValue {
        let caller = params[0].string

        let arg1 = params.dropFirst().first
        let arg2 = params.dropFirst(2).first
        let arg3 = params.dropFirst(3).first

        if methodName == "setParam" {
            guard let key = arg1?.string, let value = arg2 else {
                return XmlRpcValue(anyArray: [1,"No method for \(methodName)",-1])
            }
            parameterServer.set(param: key, value: value, notfiy: notify_param_subscribers(updates:))
            return XmlRpcValue(anyArray: [1, "parameter \(key) set", 0])
        }

        switch (methodName, arg1?.string, arg2?.string, arg3?.string) {

        // 2.1 register/unregister methods
        case ("registerService", .some(let service), .some(let service_api), .some(let caller_api)):
            return registerService(caller: caller, service: service, caller_api: caller_api, service_api: service_api)
        case ("unregisterService", .some(let service), .some(let service_api), .none):
            return unregisterService(caller: caller, service: service, service_api: service_api)

        case ("registerSubscriber", .some(let topic), .some(let topic_type), .some(let caller_api)):
            let node = Caller(id: caller, api: caller_api)
            return registerSubscriber(node: node, topic: topic, topic_type: topic_type)
        case ("unregisterSubscriber", .some(let topic), .some(let caller_api), .none):
            let node = Caller(id: caller, api: caller_api)
            return unregisterSubscriber(node: node, topic: topic)

        case ("registerPublisher", .some(let topic), .some(let topic_type), .some(let caller_api)):
            let node = Caller(id: caller, api: caller_api)
            return registerPublisher(node: node, topic: topic, topic_type: topic_type)
        case ("unregisterPublisher", .some(let topic), .some(let caller_api), .none):
            return unregisterPublisher(node: Caller(id: caller, api: caller_api), topic: topic)

        // 2.2 Name service ans system state

        case ("lookupNode", .some(let node_name), .none, .none):
            return lookupNode(caller: caller, name: node_name)
        case ("getPublishedTopics", .some(let subgraph), .none, .none):
            return getPublishedTopics(id: caller, subgraph: subgraph)
        case ("getTopicTypes", .none, .none, .none):
            return getTopicTypes(id: caller)
        case ("getSystemState", .none, .none, .none):
            return getSystemState(id: caller)
        case ("getUri", .none, .none, .none):
            return XmlRpcValue(anyArray: [1, "", 1])
        case ("lookupService", .some(let service), .none, .none):
            return lookupService(caller: caller, service: service)

        case ("getPid", .none, .none, .none):
            return getPid(id: caller)
        case ("getParam", .some(let name), .none, .none):
            return getParam(id: caller, name: name)

        // Parameter server

        case ("deleteParam", .some(let key), .none, .none):
            return deleteParam(caller_id: caller, key: key )
        case ("hasParam", .some(let key), .none, .none):
            return hasParameter(caller: caller, key: key)
        case ("searchParam", .some(let key), .none, .none):
            return searchParameter(caller: caller, key: key)
        case ("getParamNames", .none, .none, .none):
            let names = parameterServer.getAllNames().sorted()
            return XmlRpcValue(anyArray: [1, "parameter names", names])

        case ("subscribeParam", .some(let caller_api), .some(let key), .none):
            let _ = parameterServer.subscribe(parameter: key, node: Caller(id: caller, api: caller_api))
            return XmlRpcValue(anyArray: [1, "Subscribed to parameter \(key)", 1])

        case ("unsubscribeParam", .some(let caller_api), .some(let key), .none):
            let key = resolve(name: key, nameSpace: caller)
            let _ = parameterServer.unsubscribe(parameter: key, node: Caller(id: caller, api: caller_api))
            return XmlRpcValue(anyArray: [1, "Unsubscribed to parameter \(key)", 1])

        default:
            logger.error("No method for \(methodName)")
            return XmlRpcValue(anyArray: [1,"No method for \(methodName)",-1])
        }

    }


    // MARK: 2.1 register/unregister methods

    func registerService(caller: String, service: String, caller_api: String, service_api: String)  -> XmlRpcValue {
        let node = Caller(id: caller, api: caller_api)
        rm.register(service: service, node: node, service_api: service_api)
        let message = "registered [\(caller)] as provider of [\(service)]"
        logger.info("+SERVICE [\(service)] \(caller) \(service_api)")
        return XmlRpcValue(anyArray: [1,message,1])

    }

    func unregisterService(caller: String, service: String, service_api: String)  -> XmlRpcValue {
        let ret = rm.unregister(service: service, caller_id: caller, service_api: service_api)
        logger.info("-SERVICE [\(service)] \(caller) \(service_api)")
        return ret.value
    }


    func lookupService(caller: String, service: String)  -> XmlRpcValue {
        let service = resolve(name: service, nameSpace: caller)
        if let caller = rm.lookupService(key: service) {
            return XmlRpcValue(anyArray: [1, "rosrpc URI: [\(caller.api)]", caller.api])
        } else {
            return XmlRpcValue(anyArray: [-1, "no provider", ""])
        }
    }

    func registerPublisher(node: Caller, topic: String, topic_type: String) -> XmlRpcValue {
        rm.register(publisher: node, topic: topic)
        if topic_type != "*" || !topics.keys.contains(topic) {
            topics[topic] = topic_type
        }
        let pub_uris = rm.publishers(for: topic)
        let sub_uris = rm.subscribers(for: topic)
        notify_topic_subscribers(topic: topic, pub: pub_uris, sub: sub_uris)
        let message = "registered [\(node.id)] as provider of [\(topic)]"
        logger.info("+PUB [\(topic)] \(node)")
        return XmlRpcValue(anyArray: [1, message, sub_uris])
    }

    func unregisterPublisher(node: Caller, topic: String) -> XmlRpcValue {
        let retval = rm.unregister(publisher: node, topic: topic)

        if case .success = retval {
            let pub_uris = rm.publishers(for: topic)
            let sub_uris = rm.subscribers(for: topic)
            notify_topic_subscribers(topic: topic, pub: pub_uris, sub: sub_uris)
            logger.info("-PUB [\(topic)] \(node.id) \(node.api)")
        }


        return retval.value
    }

    func registerSubscriber(node: Caller, topic: String, topic_type: String) -> XmlRpcValue {
        rm.register(subscriber: node, topic: topic)
        if !topics.keys.contains(topic) && topic != "*" {
            topics[topic] = topic_type
        }
        let pub_uris = rm.publishers(for: topic)
        logger.info("+SUB [\(topic)] \(node)")
        return XmlRpcValue(anyArray: [1,"Subscribed to [\(topic)]",pub_uris])
    }
    
    func unregisterSubscriber(node: Caller, topic: String) -> XmlRpcValue {
        let ret = rm.unregister(subscriber: node, topic: topic)
        logger.info("-SUB [\(topic)] \(node)")
        return ret.value
    }


    // MARK: 2.2 Name service ans system state


    func getSystemState(id: String) -> XmlRpcValue {
        let state = rm.state()
        let xmlstate = XmlRpcValue(anyArray: state)
        return XmlRpcValue(anyArray: [1,"current system state",xmlstate])

    }

    func getTopicTypes(id: String) -> XmlRpcValue {
        let topic_types = topics.map { [$0.key,$0.value] }
        return XmlRpcValue(anyArray: [1, "current topics", topic_types])
    }

    /**
     Get list of topics that can be subscribed to. This does not return topics that have no publishers.
     See `getSystemState()` to get more comprehensive list.
    */

    func getPublishedTopics(id: String, subgraph: String) -> XmlRpcValue {

        let topic_types = topics.map { [$0.key,$0.value] }
        return XmlRpcValue(anyArray: [1, "current topics", topic_types])
    }

    func getPid(id: String) -> XmlRpcValue {
        let pid = getpid()
        return XmlRpcValue(anyArray: [1,"",pid])
    }

    func deleteParam(caller_id: String, key: String) -> XmlRpcValue {
        let key = resolve(name: key, nameSpace: caller_id)
        let _ = parameterServer.delete(param: key, notfiy: notify_param_subscribers(updates:))
        return XmlRpcValue(anyArray: [1, "parameter \(key) deleted", 0])
    }

    func getParam(id: String, name: String) -> XmlRpcValue {
        if let value = parameterServer.getValueFor(param: name) {
            return XmlRpcValue(anyArray: [1,"Parameter value",value.values])
        }
        return XmlRpcValue(anyArray: [-1,"Parameter [\(name)] is not set",0])
    }

    func hasParameter(caller: String, key: String) -> XmlRpcValue {
        let has = parameterServer.has(parameter: key)
        return XmlRpcValue(anyArray: [1,key,has])
    }

    /**
    Search for matching parameter key for search param
    key. Search for key starts at ns and proceeds upwards to
    the root. As such, search_param should only be called with a
    relative parameter name.

    search_param's behavior is to search for the first partial match.
    For example, imagine that there are two 'robot_description' parameters:

    - /robot_description
    -   /robot_description/arm
    -   /robot_description/base

    - /pr2/robot_description
    -   /pr2/robot_description/base

    If I start in the namespace /pr2/foo and search for
    'robot_description', search_param will match
    /pr2/robot_description. If I search for 'robot_description/arm'
    it will return /pr2/robot_description/arm, even though that
    parameter does not exist (yet).
 */

    func searchParameter(caller: String, key: String) -> XmlRpcValue {
        if let search_key = parameterServer.search(namespace: caller, param: key) {
            return XmlRpcValue(anyArray: [1, "Found [\(search_key)]", search_key])
        } else {
            return XmlRpcValue(anyArray: [-1, "Cannot find parameter [\(key)] in an upwards search", ""])
        }
    }



    func lookupNode(caller: String, name: String) -> XmlRpcValue {
        if let node = rm.getNode(caller: caller, name: name) {
            return XmlRpcValue(anyArray: [1, "node api", node.api])
        } else {
            return XmlRpcValue(anyArray: [-1, "unknown node \(name)", ""])
        }
    }

    // MARK: Notifications

    private func notify_topic_subscribers(topic: String, pub: [String], sub: [String]) {
        let valid = sub.compactMap { API(api: $0) }
        let msg = XmlRpcValue(anyArray: ["/master",topic,pub])

        valid.forEach { api in
            let start = DispatchTime.now()
            let client = nio.Master(group: threadGroup)
                            .connect(host: api.host, port: Int(api.port))

            client.whenSuccess { client in
                let result = client.send(method: "publisherUpdate", request: msg)

                result.whenComplete { result in
                    let msg = "publisherUpdate[\(topic)] -> \(api) \(pub)"
                    switch result {
                    case .success(let response):
                        let elapsed = DispatchTime.now().uptimeNanoseconds - start.uptimeNanoseconds
                        let time = elapsed / 1_000_000
                        logger.debug("\(msg): msec=\(time), result=\(response)")
                    case .failure(let error):
                        logger.debug("Error in publisherUpdate to \(api) returned \(error.localizedDescription)")

                    }
                    client.disconnect().whenFailure { (error) in
                        logger.debug("\(error.localizedDescription)")
                    }
                }

            }

            client.whenFailure { error in
                logger.debug("Error in publisherUpdate to \(api) returned \(error.localizedDescription)")
            }
        }
    }

    private func notify_param_subscribers(updates: [Update<XmlRpcValue>]) {
        for update in updates {
            param_update_task(caller: update.subscriber, key: update.key, value: update.value)
        }
    }

    private func param_update_task(caller: Caller, key: String, value: XmlRpcValue?) {
        guard let api = API(api: caller.api) else {
            logger.debug("malformed caller api [\(caller.api)]")
            return
        }

        let msg = XmlRpcValue(anyArray: ["/master",key,value ?? ""])

        let client = nio.Master(group: threadGroup)
                        .connect(host: api.host, port: Int(api.port))

        client.whenSuccess { client in
            let result = client.send(method: "paramUpdate", request: msg)

            result.whenComplete { result in
                switch result {
                case .success(let response):
                    logger.debug("paramUpdate to \(api) returned \(response)")
                case .failure(let error):
                    logger.debug("Error in paramUpdate to \(api) returned \(error.localizedDescription)")
                }
                _ = client.disconnect()
            }

        }

        client.whenFailure { error in
            logger.debug("Error in publisherUpdate to \(api) returned \(error.localizedDescription)")
        }
    }

}
