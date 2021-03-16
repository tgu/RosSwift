//
//  PublisherLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//


internal protocol PublisherLink: class {
    var parent: Subscription { get }
    var connectionId: Int { get set }
    var publisherXmlrpcUri: String { get }
    var stats: Stats? { get }
    var transportHints: TransportHints { get }
    var latched: Bool { get set }
    var callerId: String { get set }
    var header: Header? { get set }
    var md5sum: String { get set }

    func dropPublisherLink()
}

    struct Stats {
        var bytesReceived: UInt = 0
        var messagesReceived: UInt = 0
        var drops: UInt = 0
    }

extension PublisherLink {

    func setHeader(header: Header) -> Bool {
        guard let newId = header["callerid"] else {
            ROS_ERROR("header did not have required element: callerid")
            return false
        }

        guard let newMd5sum = header["md5sum"] else {
            ROS_ERROR("Publisher header did not have required element: md5sum")
            return false
        }

        guard header["type"] != nil else {
            ROS_ERROR("Publisher header did not have required element: type")
            return false
        }

        callerId = newId
        md5sum = newMd5sum

        latched = false
        if let latchedString = header["latching"], latchedString == "1" {
            latched = true
        }

        connectionId = parent.ros.connectionManager.getNewConnectionID()
        self.header = header

        parent.headerReceived(link: self, header: header)

        return true
    }
}
