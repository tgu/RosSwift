//
//  IntraProcessPublisherLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-23.
//

import Atomics
import StdMsgs

final class IntraProcessPublisherLink: PublisherLink {
    let parent: Subscription
    var connectionId: Int
    let publisherXmlrpcUri: String
    var stats: Stats?
    let transportHints: TransportHints
    var latched: Bool
    var callerId: String = ""
    var header: Header?
    var md5sum: String = ""

    var publisher: IntraProcessSubscriberLink?
    let isDropped = ManagedAtomic(false)

    init(parent: Subscription, xmlrpcUri: String, transportHints: TransportHints) {
        self.parent = parent
        self.connectionId = 0
        self.publisherXmlrpcUri = xmlrpcUri
        self.transportHints = transportHints
        self.latched = false
    }

    func setPublisher(ros: Ros, publisher: IntraProcessSubscriberLink) -> Bool {
        self.publisher = publisher
        let header = Header(headers: ["callerid": ros.name,
                      "topic": parent.name,
                      "type": publisher.dataType,
                      "md5sum": publisher.md5Sum,
                      "message_definition": publisher.messageDefinition,
                      "latching": publisher.isLatching ? "1" : "0"
                    ])
        return setHeader(header: header)
    }

    func getTransportType() -> String {
        return "INTRAPROCESS"
    }

    func dropPublisherLink() {
        if isDropped.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged {
            publisher?.dropParentPublication()
            publisher = nil
            ROS_DEBUG("Connection to local publisher on topic [\(parent.name)] dropped")

            parent.remove(publisherLink: self)
        }
    }

    func handleMessage(m: SerializedMessage) {
        if isDropped.load(ordering: .relaxed) {
            return
        }

        parent.handle(message: m,
                      connectionHeader: header?.headers ?? StringStringMap(),
                      link: self)

    }

}
