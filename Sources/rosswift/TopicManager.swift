//
//  File.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import BinaryCoder
import Foundation
import StdMsgs
import rpcobject
import RosTime

enum InvalidParameterError: Error {
    case invalidParameter(String)
    
    init(_ str: String) {
        self = .invalidParameter(str)
    }
}

func md5sumsMatch(lhs: String, rhs: String) -> Bool {
    return lhs == "*" || rhs == "*" || lhs == rhs
}

internal final class TopicManager {
    
    let advertisedTopicsMutex = NSRecursiveLock()
    let subsQueue = DispatchQueue(label: "subs_mutex_")
    let shuttingDownQueue = DispatchQueue(label: "shutting_down_mutex_")
    
    var advertisedTopics = [Publication]()
    var subscriptions = [Subscription]()
    var advertisedTopicNames = SynchronizedArray<String>()
    
    var shuttingDown = false
    
    var connectionManager: ConnectionManager {
        return ros.connectionManager
    }
    
    var xmlrpcManager: XMLRPCManager {
        return ros.xmlrpcManager
    }
    
    unowned var ros: Ros!
    
    deinit {
        shutdown()
    }
    
    func start(ros: Ros) {
        self.ros = ros
        shuttingDownQueue.sync {
            shuttingDown = false
            
            xmlrpcManager.bind(function: "publisherUpdate", cb: pubUpdateCallback)
            xmlrpcManager.bind(function: "requestTopic", cb: requestTopicCallback)
            xmlrpcManager.bind(function: "getBusStats", cb: getBusStatsCallback)
            xmlrpcManager.bind(function: "getBusInfo", cb: getBusInfoCallback)
            xmlrpcManager.bind(function: "getSubscriptions", cb: getSubscriptionsCallback)
            xmlrpcManager.bind(function: "getPublications", cb: getPublicationsCallback)
            
        }
    }
    
    public func shutdown() {
        shuttingDownQueue.sync {
            if shuttingDown {
                return
            }
            
            advertisedTopicsMutex.sync {
                subsQueue.sync {
                    shuttingDown = true
                }
            }
            
            xmlrpcManager.unbind(function: "publisherUpdate")
            xmlrpcManager.unbind(function: "requestTopic")
            xmlrpcManager.unbind(function: "getBusStats")
            xmlrpcManager.unbind(function: "getBusInfo")
            xmlrpcManager.unbind(function: "getSubscriptions")
            xmlrpcManager.unbind(function: "getPublications")
            
            ROS_DEBUG("Shutting down topics...")
            ROS_DEBUG("shutting down publishers")
            
            advertisedTopicsMutex.sync {
                advertisedTopics.forEach {
                    if !$0.isDropped.load() {
                        ROS_DEBUG("shutting down \($0.name)")
                        _ = unregisterPublisher(topic: $0.name)
                    }
                    $0.dropPublication()
                }
                advertisedTopics.removeAll()
                advertisedTopicNames.removeAll()
            }
            
            ROS_DEBUG("shutting down subscribers we have \(subscriptions.count) subscriptions")
            subsQueue.sync {
                
                subscriptions.forEach {
                    ROS_DEBUG("closing down subscription to \($0.name)")
                    unregisterSubscriber(topic: $0.name)
                    $0.shutdown()
                }
                subscriptions.removeAll()
            }
            
        }
    }
    
    func pubUpdateCallback(params: XmlRpcValue) -> XmlRpcValue {
        var pubs = [String]()
        for idx in 0..<params[2].size() {
            pubs.append(params[2][idx].string)
        }
        if pubUpdate(topic: params[1].string, pubs: pubs) {
            return XmlRpcValue(anyArray: [1, "", 0])
        } else {
            return XmlRpcValue(anyArray: [0, "last error", 0])
        }
    }
    
    func requestTopicCallback(params: XmlRpcValue) -> XmlRpcValue {
        return requestTopic(topic: params[1].string, protos: params[2])
    }
    
    func requestTopic(topic: String, protos: XmlRpcValue) -> XmlRpcValue {
        ROS_DEBUG("requestTopic \(topic) with \(protos)")
        for protoIndex in 0..<protos.size() {
            let proto = protos[protoIndex]
            guard case .array = proto else {
                ROS_DEBUG("requestTopic protocol list was not a list of lists")
                return XmlRpcValue()
            }
            
            guard case .string(let protoName) = proto[0] else {
                ROS_DEBUG( "requestTopic received a protocol list in which a sublist did not start with a string")
                return XmlRpcValue()
            }
            
            if protoName == "TCPROS" {
                let tcprosParams = XmlRpcValue(array: [.init(str: "TCPROS"),
                                                       .init(str: ros.network.gHost),
                                                       .init(any: connectionManager.port)])
                let ret = XmlRpcValue(array: [.init(any: 1),
                                              .init(str: ""),
                                              tcprosParams])
                return ret
            } else {
                ROS_DEBUG( "an unsupported protocol was offered: [\(protoName)]")
            }
        }
        return XmlRpcValue()
    }
    
    func getBusStatsCallback(params: XmlRpcValue) -> XmlRpcValue {
        var response = XmlRpcValue()
        getBusStats(stats: &response)
        return XmlRpcValue(anyArray: [1, "", response])
    }
    
    func getBusInfoCallback(params: XmlRpcValue ) -> XmlRpcValue {
        let response = getBusInfo()
        return XmlRpcValue(anyArray: [1, "", response])
    }
    
    func getSubscriptionsCallback(params: XmlRpcValue ) -> XmlRpcValue {
        let response = getSubscriptions()
        return XmlRpcValue(anyArray: [1, "", response])
    }
    
    func getPublicationsCallback(params: XmlRpcValue ) -> XmlRpcValue {
        let response = getPublications()
        return XmlRpcValue(anyArray: [1, "", response])
    }
    
    func getBusStats(stats: inout XmlRpcValue) {
        ROS_ERROR("getBusStats not implemented")
    }
    
    func getBusInfo() -> XmlRpcValue {
        var busInfo = [XmlRpcValue]()
        advertisedTopicsMutex.sync {
            advertisedTopics.forEach({ pub in
                busInfo.append(contentsOf: pub.getInfo() )
            })
        }
        
        subsQueue.sync {
            subscriptions.forEach({ sub in
                busInfo.append(contentsOf: sub.getInfo() )
            })
        }
        
        return XmlRpcValue(array: busInfo)
    }
    
    func getSubscriptions() -> XmlRpcValue {
        let topics = subscriptions.map { sub -> XmlRpcValue in
            XmlRpcValue(anyArray: [sub.name, sub.datatype])
        }
        return XmlRpcValue(anyArray: topics)
    }
    
    func getPublications() -> XmlRpcValue {
        let topics = advertisedTopics.map { pub -> XmlRpcValue in
            XmlRpcValue(anyArray: [pub.name, pub.datatype])
        }
        return XmlRpcValue(anyArray: topics)
    }
    
    func pubUpdate(topic: String, pubs: [String]) -> Bool {
        let sub: Subscription? = subsQueue.sync {
            if shuttingDown {
                return nil
            }
            ROS_DEBUG("Received update for topic [\(topic)] (\(pubs.count) publishers)")
            return subscriptions.first(where: { s -> Bool in
                !s.dropped.load() && s.name == topic
            })
        }
        
        if let s = sub {
            return s.pubUpdate(newPubs: pubs)
        } else {
            ROS_DEBUG("got a request for updating publishers of topic" +
                        " \(topic), but I don't have any subscribers to that topic.")
        }
        
        return false
    }
    
    func lookupPublication(topic: String) -> Publication? {
        return advertisedTopicsMutex.sync {
            lookupPublicationWithoutLock(topic: topic)
        }
    }
    
    func getAdvertised() -> [String] {
        return advertisedTopicsMutex.sync {
            return advertisedTopicNames.all()
        }
    }
    
    func getSubscribed() -> [String] {
        return subsQueue.sync {
            return subscriptions.map { $0.name }
        }
    }
    
    func unregisterPublisher(topic: String) -> Bool {
        ROS_DEBUG("unregister publisher \(topic)")
        let args = XmlRpcValue(anyArray: [ros.name, topic, xmlrpcManager.serverURI])
        do {
            let response = try ros.master.execute(method: "unregisterPublisher", request: args).wait()
            ROS_DEBUG("response = \(response)")
        } catch {
            ROS_ERROR("Error during unregisterPublisher \(error)")
        }
        
        return true
    }
    
    /// if it finds a pre-existing subscription to the same topic and of the
    /// same message type, it appends the Functor to the callback vector for
    /// that subscription. otherwise, it returns false, indicating that a new
    /// subscription needs to be created.
    
    func addSubCallback<M: Message>(ops: SubscribeOptions<M>) -> Bool {
        if shuttingDown {
            return false
        }
        
        guard let sub = subscriptions.first(where: {
            !$0.dropped.load() && $0.name == ops.topic
        }) else {
            return false
        }
        
        guard md5sumsMatch(lhs: M.md5sum, rhs: sub.md5sum) else {
            fatalError("Tried to subscribe to a topic with the same name" +
                        " but different md5sum as a topic that was already subscribed" +
                        " [\(M.datatype)/\(M.md5sum) vs. \(sub.datatype)/\(sub.md5sum)]")
        }
        
        return sub.add(callback: ops.helper,
                       md5: M.md5sum,
                       queue: ops.callbackQueue,
                       queueSize: ops.queueSize,
                       trackedObject: ops.trackedObject,
                       allowConcurrentCallbacks: ops.allowConcurrentCallbacks)
        
    }
    
    func subscribeWith<M: Message>(options: SubscribeOptions<M>) -> Bool {
        var ok = true
        do {
            try subsQueue.sync {
                
                if addSubCallback(ops: options) {
                    return
                }
                
                if shuttingDown {
                    ok = false
                    return
                }
                
                if M.md5sum.isEmpty {
                    throw InvalidParameterError("Subscribing to topic [\(options.topic)] with an empty md5sum")
                }
                
                if M.datatype.isEmpty {
                    throw InvalidParameterError("Subscribing to topic [\(options.topic)] with an empty datatype")
                }
                
                let md5sum = M.md5sum
                let datatype = M.datatype
                
                let sub = Subscription(ros: ros,
                                       name: options.topic,
                                       md5sum: md5sum,
                                       datatype: datatype,
                                       transportHints: options.transportHints)
                _ = sub.add(callback: options.helper,
                            md5: M.md5sum,
                            queue: options.callbackQueue,
                            queueSize: options.queueSize,
                            trackedObject: options.trackedObject,
                            allowConcurrentCallbacks: options.allowConcurrentCallbacks)
                
                if !registerSubscriber(s: sub, datatype: M.datatype) {
                    ROS_DEBUG("couldn't register subscriber on topic [\(options.topic)")
                    sub.shutdown()
                    ok = false
                    return
                }
                
                subscriptions.append(sub)
            }
        } catch {
            ROS_ERROR(error.localizedDescription)
            return false
        }
        return ok
    }
    
    func unsubscribe(topic: String, helper: SubscriptionCallbackHelper) -> Bool {
        var sub: Subscription?
        subsQueue.sync {
            if !shuttingDown {
                sub = subscriptions.first(where: { $0.name == topic })
            }
        }
        
        guard let s = sub else {
            return false
        }
        
        s.remove(callback: helper)
        
        if s.callbacks.isEmpty {
            // nobody is left. blow away the subscription.
            subsQueue.sync {
                subscriptions = subscriptions.filter { $0.name != topic }
                unregisterSubscriber(topic: topic)
            }
            
            s.shutdown()
        }
        
        return true
    }
    
    private func registerSubscriber(name: String, datatype: String) -> XmlRpcValue {
        let args = XmlRpcValue(anyArray: [ros.name, name, datatype, xmlrpcManager.serverURI])
        
        var payload = XmlRpcValue()
        do {
            payload = try ros.master.execute(method: "registerSubscriber", request: args).wait()
        } catch {
            ROS_ERROR("registerSubscriber \(error)")
        }
        return payload
    }
    
    func registerSubscriber(s: Subscription, datatype: String) -> Bool {
        let payload = registerSubscriber(name: s.name, datatype: datatype)
        guard payload.valid() else {
            return false
        }
        
        let pubUris = payload.map { $0.string }.filter { $0 != xmlrpcManager.serverURI }
        
        // Figure out if we have a local publisher
        
        let pub = advertisedTopicsMutex.sync {
            advertisedTopics.first(where: {$0.name == s.name && !$0.isDropped.load()})
        }
        
        if let localPub = pub {
            guard md5sumsMatch(lhs: localPub.md5sum, rhs: s.md5sum) else {
                ROS_DEBUG("md5sum mismatch making local subscription to topic \(s.name).")
                ROS_DEBUG("Subscriber expects type \(s.datatype), md5sum \(s.md5sum)")
                ROS_DEBUG("Publisher provides type \(localPub.datatype), md5sum \(localPub.md5sum)")
                return false
            }
            
            s.add(ros: ros, localConnection: localPub)
        }
        
        _ = s.pubUpdate(newPubs: pubUris)
        return true
    }
    
    func unregisterSubscriber(topic: String) {
        let args = XmlRpcValue(anyArray: [ros.name, topic, xmlrpcManager.serverURI])
        let response = ros.master.execute(method: "unregisterSubscriber", request: args)
        response.whenFailure { error in
            ROS_ERROR("Couldn't unregister subscriber for topic [\(topic)]: \(error)")
        }
    }
    
    func lookupPublicationWithoutLock(topic: String) -> Publication? {
        return advertisedTopics.first { $0.name == topic }
    }
    
    func advertise<M: Message>(ops: AdvertiseOptions<M>, callbacks: SubscriberCallbacks) -> Bool {
        if M.datatype == "*" {
            fatalError("Advertising with * as the datatype is not allowed.  Topic [\(ops.topic)")
        }
        if M.md5sum == "*" {
            fatalError("Advertising with * as the md5sum is not allowed.  Topic [\(ops.topic)]")
        }
        if M.md5sum.isEmpty {
            fatalError("Advertising on topic [\(ops.topic)] with an empty md5sum")
        }
        if M.datatype.isEmpty {
            fatalError("Advertising on topic [\(ops.topic)] with an empty datatype")
        }
        if M.definition.isEmpty {
            ROS_DEBUG("Advertising on topic \(ops.topic) with an empty message definition." +
                        " Some tools (e.g. rosbag) may not work correctly.")
        }
        
        if let pub = lookupPublicationWithoutLock(topic: ops.topic) {
            if pub.numCallbacks == 0 {
                //                pub.reset()
            }
            if pub.md5sum != M.md5sum {
                ROS_DEBUG("wrong md5sum")
                return false
            }
            
            pub.addCallbacks(callback: callbacks)
            
            return true
        }
        
        let pub = Publication(name: ops.topic,
                              datatype: M.datatype,
                              md5sum: M.md5sum,
                              messageDefinition: M.definition,
                              latch: ops.latch)
        
        pub.addCallbacks(callback: callbacks)
        advertisedTopicsMutex.sync {
            advertisedTopics.append(pub)
        }
        
        advertisedTopicNames.append(ops.topic)
        
        // Check whether we've already subscribed to this topic.  If so, we'll do
        // the self-subscription here, to avoid the deadlock that would happen if
        // the ROS thread later got the publisherUpdate with its own XMLRPC URI.
        // The assumption is that advertise() is called from somewhere other
        // than the ROS thread.
        
        subsQueue.sync                 {
            if let it = subscriptions.first(where: { s -> Bool in
                s.name == ops.topic && md5sumsMatch(lhs: s.md5sum, rhs: M.md5sum) && !s.dropped.load()
            }) {
                DispatchQueue(label: "adding").async {
                    it.add(ros: self.ros, localConnection: pub)
                }
            }
        }
        
        let args = XmlRpcValue(strings: ros.name, ops.topic, M.datatype, xmlrpcManager.serverURI)
        var payload = XmlRpcValue()
        do {
            payload = try ros.master.execute(method: "registerPublisher", request: args).wait()
        } catch {
            ROS_ERROR("registerPublisher \(error)")
        }
        ROS_DEBUG(payload.description)
        
        return true
    }
    
    func unadvertisePublisher(topic: String, callbacks: SubscriberCallbacks?) -> Bool {
        var pub: Publication?
        advertisedTopicsMutex.sync {
            if !shuttingDown {
                pub = advertisedTopics.first(where: { $0.name == topic && !$0.isDropped.load() })
            }
        }
        
        guard let p = pub else {
            return false
        }
        
        if let c = callbacks {
            p.removeCallbacks(callback: c)
        }
        
        advertisedTopicsMutex.sync {
            if p.numCallbacks == 0 {
                _ = unregisterPublisher(topic: p.name)
                p.dropPublication()
                advertisedTopics.removeAll(where: { $0.name == topic && !$0.isDropped.load() })
                advertisedTopicNames.remove(where: { $0 == topic })
            }
        }
        
        return true
    }
    
    func publish(topic: String, message: Message) {
        if shuttingDown {
            return
        }
        
        advertisedTopicsMutex.sync {
            if let p = lookupPublicationWithoutLock(topic: topic) {
                let seq = p.incrementSequence()
                if var msgWithHeader = message as? MessageWithHeader {
                    msgWithHeader.header.seq = seq
                    msgWithHeader.header.stamp = Time.now
                    let msg = SerializedMessage(msg: msgWithHeader)
                    publishSerialized(pub: p, serMsg: msg)
                } else {
                    let msg = SerializedMessage(msg: message)
                    publishSerialized(pub: p, serMsg: msg)
                }
            }
        }
    }
    
    private func publishSerialized(pub: Publication, serMsg: SerializedMessage) {
        if pub.hasSubscribers() || pub.isLatching() {
            pub.publish(msg: serMsg)
        }
    }
    
    private func incrementSequence(topic: String) -> UInt32 {
        return lookupPublication(topic: topic)?.incrementSequence() ?? 0
    }
    
    func isLatched(topic: String) -> Bool {
        if let pub = lookupPublication(topic: topic) {
            return pub.isLatched()
        }
        return false
    }
    
    func getNumPublishers(topic: String) -> Int {
        if shuttingDown {
            return 0
        }
        
        return subsQueue.sync {
            if let sub = subscriptions.first(where: { sub -> Bool in
                !sub.dropped.load() && sub.name == topic
            }) {
                return sub.getNumPublishers()
            }
            return 0
        }
    }
    
    func getNumSubscribers(topic: String) -> Int {
        if shuttingDown {
            return 0
        }
        
        return advertisedTopicsMutex.sync {
            if let p = lookupPublicationWithoutLock(topic: topic) {
                return p.getNumSubscribers()
            } else {
                return 0
            }
        }
    }
    
    func getNumSubscriptions() -> Int {
        return subsQueue.sync {
            subscriptions.count
        }
    }
    
}



@available(swift 5.1)
@propertyWrapper
public struct RosPublished<Value: Message> {
    public let topic: String
    private unowned var node: NodeHandle!
    private var cachedValue: Value?
    private var publisher: Publisher?
    
    public init(topic: String, node: NodeHandle) {
        self.topic = topic
        self.node = node
        self.publisher = node.advertise(topic: topic, message: Value.self )
    }
    
    public var wrappedValue: Value? {
        get {
            return cachedValue
        }
        
        set {
            cachedValue = newValue
            if let value = cachedValue {
                publisher?.publish(message: value)
            }
        }
    }
}
