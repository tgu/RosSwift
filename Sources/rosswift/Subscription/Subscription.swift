//
//  Subscription.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation
import Atomics
import RosTime
import StdMsgs
import rpcobject
import Synchronization

protocol TransportUDP {}

internal final class Subscription: Sendable {
    struct CallBackInfo {
        let callbackQueue: AsyncCallbackQueue
        let helper: SubscriptionCallbackHelper
        let subscriptionQueue: SubscriptionQueue
        let hasTrackedObject: Bool
        weak var trackedObject: TrackableObject?
        
        init(queue: AsyncCallbackQueue,
             helper: SubscriptionCallbackHelper,
             subscriptionQueue: SubscriptionQueue,
             hasTrackedObject: Bool,
             trackedObject: TrackableObject?) {
            self.callbackQueue = queue
            self.helper = helper
            self.subscriptionQueue = subscriptionQueue
            self.hasTrackedObject = hasTrackedObject
            self.trackedObject = trackedObject
        }
    }
    
    struct LatchInfo {
        let message: SerializedMessage
        unowned var link: PublisherLink
        let connectionHeader: [String: String]
        let receiptTime: Time
    }
    
    let datatype: String
    let md5sum: Mutex<String>
    let name: String
    let callbacks = SynchronizedArray<CallBackInfo>()
    let dropped = ManagedAtomic(false)
    let isShuttingDown = ManagedAtomic(false)
    let publisherLinks = SynchronizedArray<PublisherLink>()
    let transportHints: TransportHints
    let latchedMessages = Mutex([UUID: LatchInfo]())
    
    init(ros: Ros, name: String, md5sum: String, datatype: String, transportHints: TransportHints) {
        self.name = name
        self.datatype = datatype
        self.md5sum = Mutex(md5sum)
        self.transportHints = transportHints
    }
    
    func shutdown() {
        if isShuttingDown.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged {
            dropSubscription()
        }
    }
    
    func dropSubscription() {
        if dropped.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged {
            dropAllConnections()
        }
    }
    
    func dropAllConnections() {
        let localSubscribers = publisherLinks.all()
        publisherLinks.removeAll()
        for sub in localSubscribers {
            sub.dropPublisherLink()
        }
    }
    
    func headerReceived(link: PublisherLink, header: Header) {
        md5sum.withLock {
            if $0 == "*" {
                $0 = link.md5sum
            }
        }
    }
    
    func getNumPublishers() -> Int {
        return publisherLinks.count
    }
    
    func add(callback: SubscriptionCallbackHelper,
             md5: String,
             queue: AsyncCallbackQueue,
             queueSize: UInt32,
             trackedObject: TrackableObject?,
             allowConcurrentCallbacks: Bool) async -> Bool {
        
        md5sum.withLock {
            if $0 == "*" && md5 != "*" {
                $0 = md5
            }
        }
        
        if md5 != "*" && md5 != md5sum.withLock({ $0 }) {
            return false
        }
        
        let subQueue = SubscriptionQueue(topic: name, queueSize: queueSize, allowConcurrentCallbacks: allowConcurrentCallbacks)
        let info = CallBackInfo(queue: queue,
                                helper: callback,
                                subscriptionQueue: subQueue,
                                hasTrackedObject: trackedObject != nil,
                                trackedObject: trackedObject)
        
        callbacks.append(info)

        // Replay any cached latched messages to the newly-added callback so
        // late subscribers see the most recent value from each latched
        // publisher. Awaited (not detached) so the message is enqueued before
        // this returns — making post-subscribe() delivery deterministic.
        let latchedSnapshot = latchedMessages.withLock { $0 }
        if !latchedSnapshot.isEmpty {
            let latchedLinks = publisherLinks.filter { $0.latched }
            for pl in latchedLinks {
                guard let latchedMessage = latchedSnapshot[pl.connectionId] else { continue }
                let deserializer = MessageDeserializer(
                    helper: info.helper,
                    m: latchedMessage.message,
                    header: latchedMessage.connectionHeader)
                _ = await info.subscriptionQueue.push(helper: info.helper,
                                                      deserializer: deserializer,
                                                      hasTrackedObject: info.hasTrackedObject,
                                                      trackedObject: info.trackedObject,
                                                      receiptTime: latchedMessage.receiptTime)
                await info.callbackQueue.addCallback(callback: info.subscriptionQueue)
            }
        }

        return true
    }
    
    func add(rosName: String, localConnection: Publication, serverURI: String) async {
        if dropped.load(ordering: .relaxed) {
            return
        }

        ROS_DEBUG("Creating intraprocess link for topic [\(name)]")

        let pubLink = IntraProcessPublisherLink(parent: self, xmlrpcUri: serverURI, transportHints: transportHints)
        let subLink = IntraProcessSubscriberLink(desitnationName: rosName, parent: localConnection, subscriber: pubLink)
        _ = pubLink.setPublisher(callerId: name, publisher: subLink)

        publisherLinks.append(pubLink)
        await localConnection.addSubscriberLink(subLink)
    }
    
    func pubUpdate(rosName: String, newPubs: [String], serverURI: String, master: Master) -> Bool {
        if isShuttingDown.load(ordering: .relaxed) || dropped.load(ordering: .relaxed) {
            return false
        }
        
        var retval = true
        
        var msg = newPubs.joined(separator: ", ")
        msg += " already have these connections: "
        msg += publisherLinks.compactMap { $0.publisherXmlrpcUri }.joined(separator: ", ")
        ROS_DEBUG("Publisher update for [\(self.name)]: \(msg)")
        
        var additions = [String]()
        var subtractions = [PublisherLink]()
        
        publisherLinks.forEach { link in
            let pub = newPubs.first(where: {
                urisEqual(uri: $0, uri2: link.publisherXmlrpcUri)
            })
            if pub == nil {
                subtractions.append(link)
                return
            }
        }
        
        newPubs.forEach { newPub in
            let pub = publisherLinks.first(where: {
                urisEqual(uri: newPub, uri2: $0.publisherXmlrpcUri )
            })
            if pub == nil {
                additions.append(newPub)
            }
            
        }
        
        for link in subtractions {
            let uri = link.publisherXmlrpcUri
            if uri != serverURI {
                ROS_DEBUG("Disconnecting from publisher [\(link.callerId)] of topic [\(self.name)] at [\(serverURI)]")
                link.dropPublisherLink()
            } else {
                ROS_DEBUG("Disconnect: skipping myself for topic [\(self.name)]")
            }
        }
        
        additions.forEach {
            if $0 != serverURI {
                retval = retval && negotiate(rosName: rosName, connection: $0, master: master)
            } else {
                ROS_DEBUG("Skipping myself (\(self.name), \(serverURI)")
            }
        }
        return retval
    }
    
    func remove(callback: SubscriptionCallbackHelper) {
        if let index = callbacks.firstIndex(where: { $0.helper.id == callback.id }) {
            callbacks.remove(at: index)
        }
    }
    
    @discardableResult
    func handle(message: SerializedMessage, connectionHeader: StringStringMap, link: PublisherLink) async -> Int {
        
        var drops = 0
        let receiptTime = Time.now
        
        for info in callbacks.all() {
            let deserializer = MessageDeserializer(helper: info.helper, m: message, header: connectionHeader)
            let wasFull = await info.subscriptionQueue.push(helper: info.helper,
                                                            deserializer: deserializer,
                                                            hasTrackedObject: info.hasTrackedObject,
                                                            trackedObject: info.trackedObject,
                                                            receiptTime: receiptTime)
            if wasFull {
                drops += 1
            } else {
                await info.callbackQueue.addCallback(callback: info.subscriptionQueue)
            }
        }
        
        if link.latched {
            let info = LatchInfo(message: message,
                                 link: link,
                                 connectionHeader: connectionHeader,
                                 receiptTime: receiptTime)
            latchedMessages.withLock{$0[link.connectionId] = info}
        }
        
        return drops
    }
    
    func remove(publisherLink: PublisherLink) {
        if let index = publisherLinks.index(where: { publisherLink === $0 }) {
            publisherLinks.remove(at: index)
        }
        if publisherLink.latched {
            _ = latchedMessages.withLock{$0.removeValue(forKey: publisherLink.connectionId)}
        }
    }
    
    func negotiate(rosName: String, connection uri: String, master: Master) -> Bool {
        
        var tcprosArray = [String]()
        var protosArray = [[String]]()
        
        var transports = transportHints.getTransports()
        if transports.isEmpty {
            // Default to TCP when no transport was hinted.
            transports = ["TCP"]
        }
        
        transports.forEach {
            if $0 == "TCP" {
                tcprosArray.append("TCPROS")
                protosArray.append(tcprosArray)
            } else {
                ROS_WARNING("Unsupported transport type hinted: \($0), skipping")
            }
        }
        
        let params = XmlRpcValue(anyArray: [name, name, protosArray])
        
        guard let url = URL(string: uri), let peerHost = url.host, let peerPort = url.port else {
            ROS_ERROR("Bad xml-rpc URI: [\(uri)]")
            return false
        }
        
        Task {
            do {
                let payload = try await master.execute(method: "requestTopic",
                                                       request: params,
                                                       host: peerHost,
                                                       port: UInt16(peerPort))
                guard payload.size() == 3,
                      case .string(let pubHost) = payload[1],
                      case .int(let pubPort) = payload[2] else {
                    ROS_DEBUG("publisher implements TCPROS, but the parameters aren't string,int")
                    return
                }

                ROS_DEBUG("Connecting via tcpros to topic [\(self.name)] at host [\(pubHost):\(pubPort)]")

                let connection = InboundConnection(parent: self, host: pubHost, port: pubPort)
                let pubLink = TransportPublisherLink(parent: self, xmlrpcUri: uri, transportHints: self.transportHints)
                await pubLink.initialize(rosName: rosName, connection: connection)
                // The subscription may have been torn down while we were
                // setting up the link; if so, drop it instead of letting it
                // escape into a publisherLinks array that's already drained.
                if self.dropped.load(ordering: .relaxed) || self.isShuttingDown.load(ordering: .relaxed) {
                    pubLink.dropPublisherLink()
                    return
                }
                self.publisherLinks.append(pubLink)
                ROS_DEBUG("Connected to publisher of topic [\(self.name)] at [\(pubHost):\(pubPort)]")
            } catch {
                ROS_ERROR("requestTopic failed: \(error)")
            }
        }

        ROS_DEBUG("Began asynchronous xmlrpc connection to [\(peerHost):\(peerPort)]")

        return true
    }
    
    func getInfo() -> [XmlRpcValue] {
        var ret = [XmlRpcValue]()
        publisherLinks.forEach({ pub in
            let info: [Any] = [
                pub.connectionId,
                pub.publisherXmlrpcUri,
                "i",
                "TCPROS",
                name,
                true,
                "transport info"]
            ret.append(XmlRpcValue(anyArray: info))
        })
        
        return ret
    }
    
}

func urisEqual(uri: String, uri2: String) -> Bool {
    let url1 = URL(string: uri)
    let url2 = URL(string: uri2)
    return url1 == url2
}
