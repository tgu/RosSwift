//
//  IntraProcessPublisherLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-23.
//

import Foundation
import StdMsgs
import NIOConcurrencyHelpers


final class IntraProcessPublisherLink: PublisherLink {
    var publisher_ : IntraProcessSubscriberLink?
    var dropped_ = Atomic<Bool>(value: false)

    override init(parent: Subscription, xmlrpcUri: String, transportHints: TransportHints) {
        super.init(parent: parent, xmlrpcUri: xmlrpcUri, transportHints: transportHints)
    }

    func setPublisher(publisher: IntraProcessSubscriberLink) -> Bool {
        publisher_ = publisher
        let header = Header()

        header.read_map_ = ["callerid": Ros.this_node.getName(),
                      "topic": parent.name,
                      "type": publisher.getDataType(),
                      "md5sum": publisher.getMD5Sum(),
                      "message_definition": publisher.getMessageDefinition(),
                      "latching": publisher.isLatching() ? "1" : "0"
                    ]
        return setHeader(header: header)
    }

    func getTransportType() -> String {
        return "INTRAPROCESS"
    }

    func getTransportInfo() -> String {
        return getTransportType()
    }

    override func drop() {
        if dropped_.compareAndExchange(expected: false, desired: true) {
            publisher_?.drop()
            publisher_ = nil
            ROS_DEBUG("Connection to local publisher on topic [\(parent.name)] dropped")

            parent.remove(publisherLink: self)
        }
    }

    func handleMessage(m: SerializedMessage, ser: Bool, nocopy: Bool) {
        if dropped_.load() {
            return
        }

        parent.handle(message: m, ser: ser, nocopy: nocopy, connection_header: header?.getValues() ?? M_string(), link: self)

    }

    func getPublishTypes(ser: inout Bool, nocopy: inout Bool, ti: String) {
        if dropped_.load() {
            ser = false
            nocopy = false
            return
        }

        parent.getPublishTypes(ser: &ser, nocopy: &nocopy, ti: ti)
    }

}
