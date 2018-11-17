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
    var publisher: IntraProcessSubscriberLink?
    var isDropped = Atomic<Bool>(value: false)

    override init(parent: Subscription, xmlrpcUri: String, transportHints: TransportHints) {
        super.init(parent: parent, xmlrpcUri: xmlrpcUri, transportHints: transportHints)
    }

    func setPublisher(publisher: IntraProcessSubscriberLink) -> Bool {
        self.publisher = publisher
        let header = Header()

        header.headers = ["callerid": Ros.ThisNode.getName(),
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

    override func drop() {
        if isDropped.compareAndExchange(expected: false, desired: true) {
            publisher?.drop()
            publisher = nil
            ROS_DEBUG("Connection to local publisher on topic [\(parent.name)] dropped")

            parent.remove(publisherLink: self)
        }
    }

    func handleMessage(m: SerializedMessage, ser: Bool, nocopy: Bool) {
        if isDropped.load() {
            return
        }

        parent.handle(message: m, ser: ser, nocopy: nocopy, connectionHeader: header?.getValues() ?? StringStringMap(), link: self)

    }

    func getPublishTypes(ser: inout Bool, nocopy: inout Bool, ti: String) {
        if isDropped.load() {
            ser = false
            nocopy = false
            return
        }

        parent.getPublishTypes(ser: &ser, nocopy: &nocopy, ti: ti)
    }

}
