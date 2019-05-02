//
//  Subscription.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation
import NIOConcurrencyHelpers
import RosTime
import StdMsgs

protocol TransportUDP {}

internal final class Subscription {
    final class CallBackInfo {
        var callbackQueue: CallbackQueueInterface
        var helper: SubscriptionCallbackHelper
        var subscriptionQueue: SubscriptionQueue
        var hasTrackedObject: Bool
        var trackedObject: AnyObject?

        init(queue: CallbackQueueInterface,
             helper: SubscriptionCallbackHelper,
             subscriptionQueue: SubscriptionQueue,
             hasTrackedObject: Bool,
             trackedObject: AnyObject?) {
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
    var md5sum: String
    let name: String
    var callbacks = [CallBackInfo]()
    let dropped = Atomic<Bool>(value: false)
    let isShuttingDown = Atomic<Bool>(value: false)
    let publisherLinks = SynchronizedArray<PublisherLink>()
    let transportHints: TransportHints
    let statistics: StatisticLogger?
    let md5sumQueue = DispatchQueue(label: "md5sumQueue")
    let callbacksQueue = DispatchQueue(label: "callbacksQueue")
    var latchedMessages = [ObjectIdentifier: LatchInfo]()
    let ros: Ros

    init(ros: Ros, name: String, md5sum: String, datatype: String, transportHints: TransportHints) {
        self.name = name
        self.datatype = datatype
        self.md5sum = md5sum
        self.transportHints = transportHints
        self.statistics = nil
        self.ros = ros
    }

    func shutdown() {
        if isShuttingDown.compareAndExchange(expected: false, desired: true) {
            drop()
        }
    }

    func drop() {
        if dropped.compareAndExchange(expected: false, desired: true) {
            dropAllConnections()
        }
    }

    func dropAllConnections() {
        let localSubscribers = publisherLinks.all()
        publisherLinks.removeAll()
        localSubscribers.forEach { $0.drop() }
    }

    func headerReceived(link: PublisherLink, header: Header) {
        md5sumQueue.sync {
            if md5sum == "*" {
                md5sum = link.md5sum
            }
        }
    }

    func getNumPublishers() -> Int {
        return publisherLinks.count
    }

    func add(callback: SubscriptionCallbackHelper,
             md5: String,
             queue: CallbackQueueInterface,
             queueSize: UInt32,
             trackedObject: AnyObject?,
             allowConcurrentCallbacks: Bool) -> Bool {

        md5sumQueue.sync {
            if md5sum == "*" && md5 != "*" {
                md5sum = md5
            }
        }

        if md5 != "*" && md5 != md5sum {
            return false
        }

        callbacksQueue.sync {
            let subQueue = SubscriptionQueue(topic: name, queueSize: queueSize, allowConcurrentCallbacks: allowConcurrentCallbacks)
            let info = CallBackInfo(queue: queue,
                                    helper: callback,
                                    subscriptionQueue: subQueue,
                                    hasTrackedObject: trackedObject != nil,
                                    trackedObject: trackedObject)

            callbacks.append(info)
            if !latchedMessages.isEmpty {
                publisherLinks.filter { $0.latched }.forEach {
                    if let latchedMessage = latchedMessages[ObjectIdentifier($0)] {
                        ROS_WARNING("latched callback not implemented")
                        ROS_WARNING(("should send \(latchedMessage)"))
                    }
                }
            }
        }

        return true
    }

    func add(ros: Ros, localConnection: Publication) {
        if dropped.load() {
            return
        }

        ROS_DEBUG("Creating intraprocess link for topic [\(name)]")

        let pubLink = IntraProcessPublisherLink(parent: self, xmlrpcUri: ros.xmlrpcManager.serverURI, transportHints: transportHints)
        let subLink = IntraProcessSubscriberLink(parent: localConnection)
        _ = pubLink.setPublisher(ros: ros, publisher: subLink)
        subLink.setSubscriber(ros: ros, subscriber: pubLink)

        publisherLinks.append(pubLink)
        localConnection.addSubscriberLink(subLink)
    }

    func pubUpdate(newPubs: [String]) -> Bool {
        if isShuttingDown.load() || dropped.load() {
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

        let serverURI = ros.xmlrpcManager.serverURI
        subtractions.forEach { link in
            let uri = link.publisherXmlrpcUri
            if uri != serverURI {
                ROS_DEBUG("Disconnecting from publisher [\(link.callerId)] of topic [\(self.name)] at [\(serverURI)]")
                link.drop()
            } else {
                ROS_DEBUG("Disconnect: skipping myself for topic [\(self.name)]")
            }
        }

        additions.forEach {
            if $0 != serverURI {
                retval = retval && negotiate(connection: $0)
            } else {
                ROS_DEBUG("Skipping myself (\(self.name), \(serverURI)")
            }
        }
        return retval
    }

    func remove(callback: SubscriptionCallbackHelper) {
        callbacksQueue.sync {
            if let index = callbacks.firstIndex(where: { $0.helper.id == callback.id }) {
                callbacks.remove(at: index)
            }
        }
    }

    @discardableResult
    func handle(message: SerializedMessage, connectionHeader: StringStringMap, link: PublisherLink) -> Int {

        var drops = 0
        let receiptTime = Time.now

        callbacksQueue.sync {
            callbacks.forEach { info in
                let deserializer = MessageDeserializer(helper: info.helper, m: message, header: connectionHeader)
                let wasFull = info.subscriptionQueue.push(helper: info.helper,
                                            deserializer: deserializer,
                                            hasTrackedObject: info.hasTrackedObject,
                                            trackedObject: info.trackedObject,
                                            receiptTime: receiptTime)
                if wasFull {
                    drops += 1
                } else {
                    info.callbackQueue.addCallback(callback: info.subscriptionQueue, ownerId: ObjectIdentifier(info).hashValue)
                }
            }
        }
        if link.latched {
            let info = LatchInfo(message: message,
                                 link: link,
                                 connectionHeader: connectionHeader,
                                 receiptTime: receiptTime)
            latchedMessages[ObjectIdentifier(link)] = info
        }

        return drops
    }

    func remove(publisherLink: PublisherLink) {
        if let index = publisherLinks.index(where: { publisherLink === $0 }) {
            publisherLinks.remove(at: index)
        }
        if publisherLink.latched {
            latchedMessages.removeValue(forKey: ObjectIdentifier(publisherLink))
        }
    }

    func negotiate(connection uri: String) -> Bool {

        var tcprosArray = [String]()
        var protosArray = [[String]]()

        var transports = transportHints.getTransports()
        if transports.isEmpty {
            _ = transportHints.reliable()
            transports = transportHints.getTransports()
        }

        transports.forEach {
            if $0 == "TCP" {
                tcprosArray.append("TCPROS")
                protosArray.append(tcprosArray)
            } else {
                ROS_WARNING("Unsupported transport type hinted: \($0), skipping")
            }
        }

        let params = XmlRpcValue(anyArray: [ros.name, name, protosArray])

        guard let url = URL(string: uri), let peerHost = url.host, let peerPort = url.port else {
            ROS_ERROR("Bad xml-rpc URI: [\(uri)]")
            return false
        }

        let resp = ros.master.execute(method: "requestTopic",
                                         request: params,
                                         host: peerHost,
                                         port: UInt16(peerPort))

        resp.whenSuccess({ payload in
            guard payload.size() == 3,
                case .string(let pubHost) = payload[1],
                case .int(let pubPort) = payload[2] else {
                    ROS_DEBUG("publisher implements TCPROS, but the parameters aren't string,int")
                    return
            }

            ROS_DEBUG("Connecting via tcpros to topic [\(self.name)] at host [\(pubHost):\(pubPort)]")

            DispatchQueue.global().async {
                let connection = InboundConnection(parent: self, host: pubHost, port: pubPort)
                let pubLink = TransportPublisherLink(parent: self, xmlrpcUri: uri, transportHints: self.transportHints)
                _ = pubLink.initialize(ros: self.ros, connection: connection)
//                self.ros.connectionManager.addConnection(connection: connection)
                self.publisherLinks.append(pubLink)
                ROS_DEBUG("Connected to publisher of topic [\(self.name)] at [\(pubHost):\(pubPort)]")
            }
        })

        resp.whenFailure { error in
            ROS_ERROR("requestTopic failed: \(error)")
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
