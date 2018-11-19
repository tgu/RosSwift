//
//  PublisherLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation

class PublisherLink {

    unowned var parent: Subscription
    final var connectionId: Int
    final let publisherXmlrpcUri: String
    final var stats: Stats?
    final let transportHints: TransportHints
    final var latched: Bool
    final var callerId: String = ""
    final var header: Header?
    final var md5sum: String = ""

    struct Stats {
        var bytesReceived: UInt = 0
        var messagesReceived: UInt = 0
        var drops: UInt = 0
    }

    init(parent: Subscription, xmlrpcUri: String, transportHints: TransportHints) {
        self.parent = parent
        self.connectionId = 0
        self.publisherXmlrpcUri = xmlrpcUri
        self.transportHints = transportHints
        self.latched = false
    }

    func drop() {
        ROS_ERROR("drop")
    }

    final func setHeader(header: Header) -> Bool {
        guard let newId = header.getValue(key: "callerid") else {
            ROS_ERROR("header did not have required element: callerid")
            return false
        }

        guard let newMd5sum = header.getValue(key: "md5sum") else {
            ROS_ERROR("Publisher header did not have required element: md5sum")
            return false
        }

        guard header.getValue(key: "type") != nil else {
            ROS_ERROR("Publisher header did not have required element: type")
            return false
        }

        callerId = newId
        md5sum = newMd5sum

        latched = false
        if let latchedString = header.getValue(key: "latching"), latchedString == "1" {
            latched = true
        }

        connectionId = Ros.ConnectionManager.instance.getNewConnectionID()
        self.header = header

        parent.headerReceived(link: self, header: header)

        return true
    }

    func getTransportInfo() -> String {
        return ""
    }

}
