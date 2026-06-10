//
//  File.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import BinaryCoder
import Foundation
import NIO
import StdMsgs
import rpcobject
import RosTime
import Atomics
import Synchronization

enum InvalidParameterError: Error {
    case invalidParameter(String)
    
    init(_ str: String) {
        self = .invalidParameter(str)
    }
}

func md5sumsMatch(lhs: String, rhs: String) -> Bool {
    return lhs == "*" || rhs == "*" || lhs == rhs
}

internal final class TopicManager: RosManager {
    
    let advertisedTopics = SynchronizedArray<Publication>()
    let subscriptions = SynchronizedArray<Subscription>()
    let advertisedTopicNames = SynchronizedArray<String>()
    
    let shuttingDown = ManagedAtomic(false)
    
    var connectionManager: ConnectionManager {
        return ros.connectionManager
    }
    
    var xmlrpcManager: XMLRPCManager {
        return ros.xmlrpcManager
    }
    
    let rosName: String
    let rosID: RosID
    var ros: Ros {
        guard let r = Ros.getGlobalRos(for: rosID) else {
            fatalError()
        }

        return r
    }

    /// Non-trapping accessor for teardown paths where the owning `Ros`
    /// may already have been deallocated.
    var tryRos: Ros? {
        return Ros.getGlobalRos(for: rosID)
    }
    
    init(rosID: RosID, rosName: String) {
        self.rosID = rosID
        self.rosName = rosName
    }
    
    deinit {
        shutdown()
    }
    
    func start() async {
        shuttingDown.store(false, ordering: .relaxed)
        let xmlrpcManager = self.xmlrpcManager
        xmlrpcManager.bind(function: "publisherUpdate", cb: pubUpdateCallback)
        xmlrpcManager.bind(function: "requestTopic", cb: requestTopicCallback)
        xmlrpcManager.bind(function: "getBusStats", cb: getBusStatsCallback)
        xmlrpcManager.bind(function: "getBusInfo", cb: getBusInfoCallback)
        xmlrpcManager.bind(function: "getSubscriptions", cb: getSubscriptionsCallback)
        xmlrpcManager.bind(function: "getPublications", cb: getPublicationsCallback)
    }
    
    public func shutdown() {
        guard let work = collectUnregistrations() else { return }
        // Sync entry point: fire-and-forget the master round-trips.
        Task { [self] in
            for topic in work.publishers { await unregisterPublisher(topic: topic) }
            for topic in work.subscribers { await unregisterSubscriber(topic: topic) }
        }
    }

    /// Async variant of `shutdown()` that awaits the in-flight master
    /// `unregister*` calls before returning. Use this from async contexts
    /// (tests, structured shutdown paths) so the master receives a clean
    /// teardown before being stopped.
    public func shutdownAsync() async {
        guard let work = collectUnregistrations() else { return }
        await withTaskGroup(of: Void.self) { group in
            for topic in work.publishers {
                group.addTask { await self.unregisterPublisher(topic: topic) }
            }
            for topic in work.subscribers {
                group.addTask { await self.unregisterSubscriber(topic: topic) }
            }
        }
    }

    /// Performs the synchronous teardown bookkeeping (unbind XML-RPC methods,
    /// drop publications, tear down subscriptions) exactly once, and returns
    /// the topic names that still need a master `unregister*` round-trip.
    /// Returns nil if shutdown was already in progress.
    private func collectUnregistrations() -> (publishers: [String], subscribers: [String])? {
        if shuttingDown.compareExchange(expected: false, desired: true, ordering: .relaxed).original {
            return nil
        }

        // The owning Ros may already be torn down (e.g. when our deinit
        // races with Ros deallocation). Skip network teardown in that case.
        let liveRos = tryRos

        if let xmlrpc = liveRos?.xmlrpcManager {
            xmlrpc.unbind(function: "publisherUpdate")
            xmlrpc.unbind(function: "requestTopic")
            xmlrpc.unbind(function: "getBusStats")
            xmlrpc.unbind(function: "getBusInfo")
            xmlrpc.unbind(function: "getSubscriptions")
            xmlrpc.unbind(function: "getPublications")
        }

        ROS_DEBUG("Shutting down topics...")
        ROS_DEBUG("shutting down publishers")

        var publishers = [String]()
        advertisedTopics.forEach {
            if !$0.isDropped.load(ordering: .relaxed) {
                ROS_DEBUG("shutting down \($0.name)")
                if liveRos != nil {
                    publishers.append($0.name)
                }
            }
            $0.dropPublication()
        }
        advertisedTopics.removeAll()
        advertisedTopicNames.removeAll()

        ROS_DEBUG("shutting down subscribers we have \(subscriptions.count) subscriptions")
        var subscribers = [String]()
        let subs = subscriptions.all()
        subscriptions.removeAll()
        for sub in subs {
            ROS_DEBUG("closing down subscription to \(sub.name)")
            if liveRos != nil {
                subscribers.append(sub.name)
            }
            sub.shutdown()
        }
        return (publishers, subscribers)
    }
    
    func pubUpdateCallback(params: XmlRpcValue) -> XmlRpcValue {
        guard case .array = params[2] else {
            ROS_ERROR("publisherUpdate: expected an array of publisher URIs, got \(params[2])")
            return XmlRpcValue(anyArray: [0, "publishers field is not an array", 0])
        }
        var pubs = [String]()
        for idx in 0..<params[2].size() {
            pubs.append(params[2][idx].string)
        }
        if pubUpdate(rosName: rosName, topic: params[1].string, pubs: pubs) {
            return XmlRpcValue(anyArray: [1, "", 0])
        } else {
            let lastError = shuttingDown.load(ordering: .relaxed) ? "Shutting down" : "Unknown Error"
            return XmlRpcValue(anyArray: [0, lastError, 0])
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
                return XmlRpc.responseInt(code: 0,
                                          msg: "requestTopic protocol list was not a list of lists",
                                          response: 0)
            }

            guard case .string(let protoName) = proto[0] else {
                ROS_DEBUG( "requestTopic received a protocol list in which a sublist did not start with a string")
                return XmlRpc.responseInt(code: 0,
                                          msg: "requestTopic protocol list contained a sublist that did not start with a string",
                                          response: 0)
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
        return XmlRpc.responseInt(code: 0,
                                  msg: "No supported protocol was offered. RosSwift only supports TCPROS.",
                                  response: 0)
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
        advertisedTopics.forEach { pub in
            busInfo.append(contentsOf: pub.getInfo() )
        }
        
        subscriptions.forEach { sub in
            busInfo.append(contentsOf: sub.getInfo() )
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
    
    func pubUpdate(rosName: String, topic: String, pubs: [String]) -> Bool {
        if shuttingDown.load(ordering: .relaxed) {
            return false
        }
        
        ROS_DEBUG("Received update for topic [\(topic)] (\(pubs.count) publishers)")
        let sub = subscriptions.first(where: { s -> Bool in
            !s.dropped.load(ordering: .relaxed) && s.name == topic
        })
        
        if let s = sub {
            return s.pubUpdate(rosName: rosName, newPubs: pubs, serverURI: ros.xmlrpcManager.serverURI, master: ros.master)
        } else {
            ROS_DEBUG("got a request for updating publishers of topic" +
                      " \(topic), but I don't have any subscribers to that topic.")
        }
        
        return false
    }
    
    func lookupPublication(topic: String) -> Publication? {
        lookupPublicationWithoutLock(topic: topic)
    }
    
    func getAdvertised() -> [String] {
        advertisedTopicNames.all()
    }
    
    func getSubscribed() -> [String] {
        subscriptions.map { $0.name }
    }
    
    func unregisterPublisher(topic: String) async {
        ROS_DEBUG("unregister publisher \(topic)")
        let args = XmlRpcValue(anyArray: [rosName, topic, xmlrpcManager.serverURI])
        do {
            let response = try await ros.master.execute(method: "unregisterPublisher", request: args)
            ROS_DEBUG("response = \(response)")
        } catch {
            if shuttingDown.load(ordering: .relaxed) {
                ROS_DEBUG("unregisterPublisher \(topic) failed during shutdown: \(error)")
            } else {
                ROS_ERROR("Error during unregisterPublisher \(error)")
            }
        }
    }
    
    /// if it finds a pre-existing subscription to the same topic and of the
    /// same message type, it appends the Functor to the callback vector for
    /// that subscription. otherwise, it returns false, indicating that a new
    /// subscription needs to be created.
    
    func addSubCallback<M: Message>(ops: SubscribeOptions<M>) -> Bool {
        if shuttingDown.load(ordering: .relaxed) {
            return false
        }
        
        guard let sub = subscriptions.first(where: {
            !$0.dropped.load(ordering: .relaxed) && $0.name == ops.topic
        }) else {
            return false
        }
        
        let subMd5 = sub.md5sum.withLock({$0})
        guard md5sumsMatch(lhs: M.md5sum, rhs: subMd5) else {
            ROS_ERROR("Tried to subscribe to a topic with the same name" +
                      " but different md5sum as a topic that was already subscribed" +
                      " [\(M.datatype)/\(M.md5sum) vs. \(sub.datatype)/\(subMd5)]")
            return false
        }
        
        return sub.add(callback: ops.helper,
                       md5: M.md5sum,
                       queue: ops.callbackQueue,
                       queueSize: ops.queueSize,
                       trackedObject: ops.trackedObject,
                       allowConcurrentCallbacks: ops.allowConcurrentCallbacks)
        
    }
    
    func subscribeWith<M: Message>(options: SubscribeOptions<M>) async -> Bool {
        if shuttingDown.load(ordering: .relaxed) {
            return false
        }
        
        if addSubCallback(ops: options) {
            return true
        }
        
        do {
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
            
            if !(await registerSubscriber(s: sub, datatype: M.datatype)) {
                ROS_DEBUG("couldn't register subscriber on topic [\(options.topic)")
                sub.shutdown()
                return false
            }
            
            subscriptions.append(sub)
        } catch {
            ROS_ERROR(error.localizedDescription)
            return false
        }
        return true
    }
    
    func unsubscribe(topic: String, helper: SubscriptionCallbackHelper) -> Bool {
        var sub: Subscription?

        if !shuttingDown.load(ordering: .relaxed) {
            sub = subscriptions.first(where: {
                !$0.dropped.load(ordering: .relaxed) && $0.name == topic
            })
        }

        guard let s = sub else {
            return false
        }
        
        s.remove(callback: helper)
        
        if s.callbacks.isEmpty {
            // nobody is left. blow away the subscription.
            subscriptions.filterSelf { $0.name != topic }
            Task { await unregisterSubscriber(topic: topic) }
            s.shutdown()
        }
        
        return true
    }
    
    private func registerSubscriber(name: String, datatype: String) async -> XmlRpcValue {
        let args = XmlRpcValue(anyArray: [rosName, name, datatype, xmlrpcManager.serverURI])

        do {
            return try await ros.master.execute(method: "registerSubscriber", request: args)
        } catch {
            ROS_ERROR("registerSubscriber \(error)")
            return XmlRpcValue()
        }
    }

    func registerSubscriber(s: Subscription, datatype: String) async -> Bool {
        let payload = await registerSubscriber(name: s.name, datatype: datatype)
        guard payload.valid() else {
            return false
        }
        
        let pubUris = payload.map { $0.string }.filter { $0 != xmlrpcManager.serverURI }
        
        // Figure out if we have a local publisher
        
        let pub = advertisedTopics.first(where: {$0.name == s.name && !$0.isDropped.load(ordering: .relaxed)})
        
        if let localPub = pub {
            let submd5 = s.md5sum.withLock({$0})
            guard md5sumsMatch(lhs: localPub.md5sum, rhs: submd5) else {
                ROS_DEBUG("md5sum mismatch making local subscription to topic \(s.name).")
                ROS_DEBUG("Subscriber expects type \(s.datatype), md5sum \(submd5)")
                ROS_DEBUG("Publisher provides type \(localPub.datatype), md5sum \(localPub.md5sum)")
                return false
            }
            
            s.add(rosName: rosName, localConnection: localPub, serverURI: ros.xmlrpcManager.serverURI)
        }
        
        _ = s.pubUpdate(rosName: rosName, newPubs: pubUris, serverURI: ros.xmlrpcManager.serverURI, master: ros.master)
        return true
    }
    
    func unregisterSubscriber(topic: String) async {
        let args = XmlRpcValue(anyArray: [rosName, topic, xmlrpcManager.serverURI])
        do {
            _ = try await ros.master.execute(method: "unregisterSubscriber", request: args)
        } catch {
            if shuttingDown.load(ordering: .relaxed) {
                ROS_DEBUG("unregisterSubscriber \(topic) failed during shutdown: \(error)")
            } else {
                ROS_ERROR("Couldn't unregister subscriber for topic [\(topic)]: \(error)")
            }
        }
    }
    
    func lookupPublicationWithoutLock(topic: String) -> Publication? {
        return advertisedTopics.first { $0.name == topic }
    }
    
    func advertise<M: Message>(ops: AdvertiseOptions<M>, callbacks: SubscriberCallbacks) async -> Bool {
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
        advertisedTopics.append(pub)
        
        advertisedTopicNames.append(ops.topic)
        
        // Check whether we've already subscribed to this topic.  If so, we'll do
        // the self-subscription here, to avoid the deadlock that would happen if
        // the ROS thread later got the publisherUpdate with its own XMLRPC URI.
        // The assumption is that advertise() is called from somewhere other
        // than the ROS thread.
        
        if let it = subscriptions.first(where: { s -> Bool in
            s.name == ops.topic && md5sumsMatch(lhs: s.md5sum.withLock({$0}), rhs: M.md5sum) && !s.dropped.load(ordering: .relaxed)
        }) {
            it.add(rosName: rosName, localConnection: pub, serverURI: ros.xmlrpcManager.serverURI)
        }
        
        let args = XmlRpcValue(strings: rosName, ops.topic, M.datatype, xmlrpcManager.serverURI)
        var payload = XmlRpcValue()
        do {
            payload = try await ros.master.execute(method: "registerPublisher", request: args)
        } catch {
            ROS_ERROR("registerPublisher \(error)")
        }
        ROS_DEBUG(payload.description)
        
        return true
    }
    
    func unadvertisePublisher(topic: String, callbacks: SubscriberCallbacks?) -> Bool {
        var pub: Publication?
        if !shuttingDown.load(ordering: .relaxed) {
            pub = advertisedTopics.first(where: { $0.name == topic && !$0.isDropped.load(ordering: .relaxed) })
        }
        
        guard let p = pub else {
            return false
        }
        
        if let c = callbacks {
            p.removeCallbacks(callback: c)
        }
        
        if p.numCallbacks == 0 {
            Task { await unregisterPublisher(topic: p.name) }
            advertisedTopics.removeAll(where: { $0 === p })
            advertisedTopicNames.remove(where: { $0 == topic })
            p.dropPublication()
        }
        
        return true
    }
    
    func publish(topic: String, message: Message) async {
        if shuttingDown.load(ordering: .relaxed) {
            return
        }
        
        if let p = lookupPublicationWithoutLock(topic: topic) {
            let seq = p.incrementSequence()
            if var msgWithHeader = message as? MessageWithHeader {
                msgWithHeader.header.seq = seq
                msgWithHeader.header.stamp = Time.now
                let msg = SerializedMessage(msg: msgWithHeader)
                await publishSerialized(pub: p, serMsg: msg)
            } else {
                let msg = SerializedMessage(msg: message)
                await publishSerialized(pub: p, serMsg: msg)
            }
        }
    }
    
    private func publishSerialized(pub: Publication, serMsg: SerializedMessage) async {
        if pub.hasSubscribers() || pub.isLatched {
            await pub.publish(msg: serMsg)
        }
    }
    
    private func incrementSequence(topic: String) -> UInt32 {
        return lookupPublication(topic: topic)?.incrementSequence() ?? 0
    }
    
    func isLatched(topic: String) -> Bool {
        if let pub = lookupPublication(topic: topic) {
            return pub.isLatched
        }
        return false
    }
    
    func getNumPublishers(topic: String) -> Int {
        if shuttingDown.load(ordering: .relaxed) {
            return 0
        }
        
        if let sub = subscriptions.first(where: { sub -> Bool in
            !sub.dropped.load(ordering: .relaxed) && sub.name == topic
        }) {
            return sub.getNumPublishers()
        }
        return 0
    }
    
    func getNumSubscribers(topic: String) -> Int {
        if shuttingDown.load(ordering: .relaxed) {
            return 0
        }
        
        if let p = lookupPublicationWithoutLock(topic: topic) {
            return p.getNumSubscribers()
        } else {
            return 0
        }
    }
    
    func getNumSubscriptions() -> Int {
        subscriptions.count
    }
    
}



@available(swift 5.1)
@propertyWrapper
public struct RosPublished<Value: Message>: ~Copyable {
    public let topic: String
    private unowned var node: NodeHandle!
    private var cachedValue: Value?
    private let publisher: SpecializedPublisher<Value>?
    
    public init(topic: String, node: NodeHandle) async {
        self.topic = topic
        self.node = node
        self.publisher = await node.advertise(topic: topic, message: Value.self )
    }
    
    public var wrappedValue: Value? {
        get {
            return cachedValue
        }
        
        set {
            cachedValue = newValue
            if let value = cachedValue {
                if let p = publisher {
                    Task{ await p.publish(message: value) }
                }
            }
        }
    }
}
