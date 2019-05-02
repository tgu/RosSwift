//
//  IntraProcessPublisherLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-23.
//

import Foundation
import NIOConcurrencyHelpers
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
    let isDropped = Atomic<Bool>(value: false)

    init(parent: Subscription, xmlrpcUri: String, transportHints: TransportHints) {
        self.parent = parent
        self.connectionId = 0
        self.publisherXmlrpcUri = xmlrpcUri
        self.transportHints = transportHints
        self.latched = false
    }

    func setPublisher(ros: Ros, publisher: IntraProcessSubscriberLink) -> Bool {
        self.publisher = publisher
        let header = Header()

        header.headers = ["callerid": ros.name,
                      "topic": parent.name,
                      "type": publisher.dataType,
                      "md5sum": publisher.md5Sum,
                      "message_definition": publisher.messageDefinition,
                      "latching": publisher.isLatching() ? "1" : "0"
                    ]
        return setHeader(ros: ros, header: header)
    }

    func getTransportType() -> String {
        return "INTRAPROCESS"
    }

    func drop() {
        if isDropped.compareAndExchange(expected: false, desired: true) {
            publisher?.drop()
            publisher = nil
            ROS_DEBUG("Connection to local publisher on topic [\(parent.name)] dropped")

            parent.remove(publisherLink: self)
        }
    }

    func handleMessage(m: SerializedMessage) {
        if isDropped.load() {
            return
        }

        parent.handle(message: m,
                      connectionHeader: header?.getValues() ?? StringStringMap(),
                      link: self)

    }

}
