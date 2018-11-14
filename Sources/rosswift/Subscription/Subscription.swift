//
//  Subscription.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation
import StdMsgs
import RosTime
import NIOConcurrencyHelpers


protocol TransportUDP {}

final class Subscription {
    struct CallBackInfo {
        var helper : SubscriptionCallbackHelper?
        var has_tracked_object : Bool
        var tracked_object : AnyObject?
    }

    struct LatchInfo {
        let message: SerializedMessage
        unowned var link: PublisherLink
        let connection_header: [String:String]
        let receipt_time: RosTime.Time
    }


    var datatype : String
    var md5sum_ : String
    var name : String
    var callbacks = [CallBackInfo]()
    var dropped = Atomic<Bool>(value: false)
    var shutting_down = Atomic<Bool>(value: false)
    var publisher_links = SynchronizedArray<PublisherLink>()
    var transport_hints = TransportHints()
    var statistics: StatisticLogger? = nil
    let md5sum_mutex = DispatchQueue(label: "md5sum_mutex")
    let callbacks_mutex = DispatchQueue(label: "callbacks_mutex")
    var latched_messages_ = [ObjectIdentifier : LatchInfo]()

    init(name: String, md5sum: String, datatype: String, transport_hints: TransportHints) {
        self.name = name
        self.datatype = datatype
        self.md5sum_  = md5sum
        self.transport_hints = transport_hints
    }

    func shutdown() {
        if shutting_down.compareAndExchange(expected: false, desired: true) {
            drop()
        }
    }

    func drop() {
        if dropped.compareAndExchange(expected: false, desired: true) {
            dropAllConnections()
        }
    }

    func dropAllConnections() {
        let localSubscribers = publisher_links.all()
        publisher_links.removeAll()
        localSubscribers.forEach { $0.drop() }
    }


    func headerReceived(link: PublisherLink, header: Header) {
        md5sum_mutex.sync {
            if md5sum_ == "*" {
                md5sum_ = link.md5sum
            }
        }
    }

    func getNumPublishers() -> Int {
        return publisher_links.count
    }


    func add(callback: SubscriptionCallbackHelper, md5sum: String, tracked_object: AnyObject?, allow_concurrent_callbacks: Bool) -> Bool {

        md5sum_mutex.sync {
            if md5sum_ == "*" && md5sum != "*" {
                md5sum_ = md5sum
            }
        }

        if md5sum != "*" && md5sum != md5sum_ {
            return false
        }


        callbacks_mutex.sync {
            let info = CallBackInfo(helper: callback, has_tracked_object: tracked_object != nil, tracked_object: tracked_object)

            callbacks.append(info)
            if !latched_messages_.isEmpty {
                publisher_links.filter{ $0.latched }.forEach {
                    if let latched_message = latched_messages_[ObjectIdentifier($0)] {
                        ROS_WARNING("latched callback not implemented")
                        ROS_WARNING(("should send \(latched_message)"))
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

        let pub_link = IntraProcessPublisherLink(parent: self, xmlrpcUri: XMLRPCManager.instance.serverURI, transportHints: transport_hints)
        let sub_link = IntraProcessSubscriberLink(parent: localConnection)
        pub_link.setPublisher(publisher: sub_link)
        sub_link.setSubscriber(subscriber: pub_link)

        publisher_links.append(pub_link)
        localConnection.addSubscriberLink(sub_link)
    }

    func pubUpdate(new_pubs: [String]) -> Bool {
        if shutting_down.load() || dropped.load()  {
            return false
        }

        var retval = true


        var ss = new_pubs.joined(separator: ", ")
        ss += " already have these connections: "
        ss += publisher_links.compactMap{ $0.publisherXmlrpcUri }.joined(separator: ", ")
        ROS_DEBUG("Publisher update for [\(self.name)]: \(ss)")

        var additions = [String]()
        var subtractions = [PublisherLink]()

        publisher_links.forEach { (pl) in
            let pub = new_pubs.first(where: {
                urisEqual(uri: $0, uri2: pl.publisherXmlrpcUri)
            })
            if pub == nil {
                subtractions.append(pl)
                return
            }
        }

        new_pubs.forEach { (np) in
            let pub = publisher_links.first(where: {
                urisEqual(uri: np, uri2: $0.publisherXmlrpcUri )
            })
            if pub == nil {
                additions.append(np)
            }

        }


        let serverURI = XMLRPCManager.instance.serverURI
        subtractions.forEach { (link) in
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
        callbacks_mutex.sync {
            if let index = callbacks.index(where: { $0.helper != nil && $0.helper!.id == callback.id }) {
                callbacks.remove(at: index)
            }
        }

    }



    func handle(message: SerializedMessage, ser: Bool, nocopy: Bool, connection_header: M_string, link:  PublisherLink) -> Int {
        callbacks_mutex.sync {}

        var drops = 0

        let receipt_time = RosTime.Time.now()

        callbacks.forEach { (info) in

            if let mes = message.message {
                info.helper?.call(msg: mes)
            } else {

                let ti = info.helper!.getTypeInfo()

                let a = nocopy && message.type_info != nil  && message.type_info! == ti
                let b = ser && (message.type_info == nil || ( ti != message.type_info!))
                if a || b {
                    let deserializer = MessageDeserializer(helper: info.helper!, m: message, header: connection_header)

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
            let li = LatchInfo(message: message, link: link, connection_header: connection_header, receipt_time: receipt_time)
            latched_messages_[ObjectIdentifier(link)] = li
        }


        return drops
    }

    func remove(publisherLink: PublisherLink) {
            if let index = publisher_links.index(where: {publisherLink === $0}) {
                publisher_links.remove(at: index)
            }
            if publisherLink.latched {
                latched_messages_.removeValue(forKey: ObjectIdentifier(publisherLink))
            }
    }

    func negotiate(connection uri: String) -> Bool {

        var tcpros_array = [String]()
        var protos_array = [[String]]()

        var transports = transport_hints.getTransports()
        if transports.isEmpty {
            let _ = transport_hints.reliable()
            transports = transport_hints.getTransports()
        }

        transports.forEach {
            if $0 == "UDP" {
                ROS_WARNING("UDP not implemented")
            } else if $0 == "TCP" {
                tcpros_array.append("TCPROS")
                protos_array.append(tcpros_array)
            } else {
                ROS_WARNING("Unsupported transport type hinted: \($0), skipping")
            }
        }


        let params = XmlRpcValue(anyArray: [Ros.this_node.getName(),name,protos_array])


        guard let url = URL(string: uri), let peer_host = url.host, let peer_port = url.port else {
            ROS_ERROR("Bad xml-rpc URI: [\(uri)]")
            return false
        }
        
        let resp = Master.shared.execute(method: "requestTopic", request: params, host: peer_host, port: UInt16(peer_port))

        resp.whenSuccess({ (payload) in
            guard payload.size() == 3,
                case .string(let pub_host) = payload[1].value,
                case .int(let pub_port) = payload[2].value else {
                    ROS_DEBUG("publisher implements TCPROS, but the parameters aren't string,int")
                    return
            }

            ROS_DEBUG("Connecting via tcpros to topic [\(self.name)] at host [\(pub_host):\(pub_port)]")

            
            DispatchQueue.global().async {
                let connection = InboundConnection(parent: self, host: pub_host, port: pub_port)
                let pub_link = TransportPublisherLink(parent: self, xmlrpcUri: uri, transportHints: self.transport_hints)
                let _ = pub_link.initialize(connection: connection)
                Ros.ConnectionManager.instance.addConnection(connection: connection)
                self.publisher_links.append(pub_link)
                ROS_DEBUG("Connected to publisher of topic [\(self.name)] at [\(pub_host):\(pub_port)]")
            }
        })

        resp.whenFailure { (error) in
            ROS_ERROR("requestTopic failed: \(error)")
        }



        ROS_DEBUG("Began asynchronous xmlrpc connection to [\(peer_host):\(peer_port)]")

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


}


func urisEqual(uri: String, uri2: String) -> Bool {
    let u1 = URL(string: uri)
    let u2 = URL(string: uri2)
    return u1 == u2
}

