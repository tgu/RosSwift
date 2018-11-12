//
//  PublisherLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation


class PublisherLink {

    unowned var parent: Subscription
    var connectionId : Int
    let publisherXmlrpcUri: String
    var stats: Stats? = nil
    let transportHints: TransportHints
    var latched: Bool
    var callerId : String = ""
    var header : Header? = nil
    var md5sum : String = ""

    class Stats {
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

    deinit {
        
    }

    func drop() {
        ROS_DEBUG("drop")
        fatalError()
    }

    func setHeader(header: Header) -> Bool
    {
        guard let new_caller_id = header.getValue(key: "callerid") else {
            ROS_ERROR("header did not have required element: callerid")
            return false
        }

        guard let new_md5sum = header.getValue(key: "md5sum") else {
            ROS_ERROR("Publisher header did not have required element: md5sum")
            return false
        }

        guard let _ = header.getValue(key: "type") else {
            ROS_ERROR("Publisher header did not have required element: type")
            return false
        }

        callerId = new_caller_id
        md5sum = new_md5sum

        latched = false
        if let latched_str = header.getValue(key: "latching"), latched_str == "1" {
            latched = true
        }

        connectionId = Ros.ConnectionManager.instance.getNewConnectionID()
        self.header = header

        parent.headerReceived(link: self, header: header)

        return true
    }

}
