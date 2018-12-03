//
//  TransportPublisherLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation
import RosTime
import StdMsgs

final class TransportPublisherLink: PublisherLink {
    var parent: Subscription
    var connectionId: Int
    let publisherXmlrpcUri: String
    var stats: Stats?
    let transportHints: TransportHints
    var latched: Bool
    var callerId: String = ""
    var header: Header?
    var md5sum: String = ""
    
    private var connection: InboundConnection?
    private var retryTimerHandle: Int32
    private var needsRetry: Bool
    private var retryPeriod = RosTime.WallDuration()
    private var nextRetry = RosTime.SteadyTime()
    private var isDropping: Bool

    init(parent: Subscription, xmlrpcUri: String, transportHints: TransportHints) {
        retryTimerHandle = -1
        needsRetry = false
        isDropping = false
        self.parent = parent
        self.connectionId = 0
        self.publisherXmlrpcUri = xmlrpcUri
        self.transportHints = transportHints
        self.latched = false
    }

    deinit {
        if retryTimerHandle != -1 {
            getInternalTimerManager().remove(timerHandle: retryTimerHandle)
        }
        connection?.drop(reason: .destructing)
    }

    func initialize(connection: InboundConnection) {
        self.connection = connection
        connection.initialize(owner: self)

        let header: StringStringMap = ["topic": parent.name,
                                     "md5sum": parent.md5sum,
                                     "callerid": Ros.ThisNode.getName(),
                                     "type": parent.datatype,
                                     "tcp_nodelay": transportHints.getTCPNoDelay() ? "1" : "0"]

        connection.writeHeader(keyVals: header).whenComplete {
            ROS_DEBUG("TransportPublisherLink: header is written")
        }
    }

    func drop() {
        isDropping = true
        parent.remove(publisherLink: self)
        connection?.drop(reason: .destructing)
        connection = nil
    }

    func getTransportInfo() -> String {
        return connection?.getTransportInfo() ?? ""
    }

}
