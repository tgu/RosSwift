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

final class Subscription {
    struct CallBackInfo {
        var helper: SubscriptionCallbackHelper?
        var hasTrackedObject: Bool
        var trackedObject: AnyObject?
    }

    struct LatchInfo {
        let message: SerializedMessage
        unowned var link: PublisherLink
        let connectionHeader: [String: String]
        let receiptTime: RosTime.Time
    }

    var datatype: String
    var md5sum: String
    var name: String
    var callbacks = [CallBackInfo]()
    var dropped = Atomic<Bool>(value: false)
    var isShuttingDown = Atomic<Bool>(value: false)
    var publisherLinks = SynchronizedArray<PublisherLink>()
    var transportHints = TransportHints()
    var statistics: StatisticLogger?
    let md5sumQueue = DispatchQueue(label: "md5sumQueue")
    let callbacksQueue = DispatchQueue(label: "callbacksQueue")
    var latchedMessages = [ObjectIdentifier: LatchInfo]()

    init(name: String, md5sum: String, datatype: String, transportHints: TransportHints) {
        self.name = name
        self.datatype = datatype
        self.md5sum = md5sum
        self.transportHints = transportHints
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

    func add(callback: SubscriptionCallbackHelper, md5: String, trackedObject: AnyObject?, allowConcurrentCallbacks: Bool) -> Bool {

        md5sumQueue.sync {
            if md5sum == "*" && md5 != "*" {
                md5sum = md5
            }
        }

        if md5 != "*" && md5 != md5sum {
            return false
        }

        callbacksQueue.sync {
            let info = CallBackInfo(helper: callback, hasTrackedObject: trackedObject != nil, trackedObject: trackedObject)

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

    func add(localConnection: Publication) {
        if dropped.load() {
            return
        }

        ROS_DEBUG("Creating intraprocess link for topic [\(name)]")

        let pubLink = IntraProcessPublisherLink(parent: self, xmlrpcUri: XMLRPCManager.instance.serverURI, transportHints: transportHints)
        let subLink = IntraProcessSubscriberLink(parent: localConnection)
        pubLink.setPublisher(publisher: subLink)
        subLink.setSubscriber(subscriber: pubLink)

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

        let serverURI = XMLRPCManager.instance.serverURI
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
            if let index = callbacks.index(where: { $0.helper != nil && $0.helper!.id == callback.id }) {
                callbacks.remove(at: index)
            }
        }
    }

    @discardableResult
    func handle(message: SerializedMessage, ser: Bool, nocopy: Bool, connectionHeader: StringStringMap, link: PublisherLink) -> Int {
        callbacksQueue.sync {}

        var drops = 0

        let receiptTime = RosTime.Time.now()

        callbacks.forEach { info in
            if let mes = message.message {
                info.helper?.call(msg: mes)
            } else {
                let ti = info.helper!.getTypeInfo()
                let a = nocopy && message.typeInfo != nil && message.typeInfo! == ti
                let b = ser && ( message.typeInfo == nil || ( ti != message.typeInfo! ))
                if a || b {
                    let deserializer = MessageDeserializer(helper: info.helper!, m: message, header: connectionHeader)
                    DispatchQueue.global().async {
                        if let msg = deserializer.deserialize() {
                            info.helper?.call(msg: msg)
                        }
                    }
                }
            }
        }

//        // measure statistics
//        statistics_.callback(connection_header, name_, link->getCallerID(), m, link->getStats().bytes_received_, receipt_time, drops > 0);
//
//        // If this link is latched, store off the message so we can immediately pass it to new subscribers later

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
            if $0 == "UDP" {
                ROS_WARNING("UDP not implemented")
            } else if $0 == "TCP" {
                tcprosArray.append("TCPROS")
                protosArray.append(tcprosArray)
            } else {
                ROS_WARNING("Unsupported transport type hinted: \($0), skipping")
            }
        }

        let params = XmlRpcValue(anyArray: [Ros.ThisNode.getName(), name, protosArray])

        guard let url = URL(string: uri), let peerHost = url.host, let peerPort = url.port else {
            ROS_ERROR("Bad xml-rpc URI: [\(uri)]")
            return false
        }

        let resp = Master.shared.execute(method: "requestTopic",
                                         request: params,
                                         host: peerHost,
                                         port: UInt16(peerPort))

        resp.whenSuccess({ payload in
            guard payload.size() == 3,
                case .string(let pubHost) = payload[1].value,
                case .int(let pubPort) = payload[2].value else {
                    ROS_DEBUG("publisher implements TCPROS, but the parameters aren't string,int")
                    return
            }

            ROS_DEBUG("Connecting via tcpros to topic [\(self.name)] at host [\(pubHost):\(pubPort)]")

            DispatchQueue.global().async {
                let connection = InboundConnection(parent: self, host: pubHost, port: pubPort)
                let pubLink = TransportPublisherLink(parent: self, xmlrpcUri: uri, transportHints: self.transportHints)
                _ = pubLink.initialize(connection: connection)
                Ros.ConnectionManager.instance.addConnection(connection: connection)
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

    func getPublishTypes(ser: inout Bool, nocopy: inout Bool, ti: String) {
        callbacks.forEach {
            if $0.helper?.getTypeInfo() == ti {
                nocopy = true
            } else {
                ser = true
            }
            if nocopy && ser {
                return
            }
        }
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
