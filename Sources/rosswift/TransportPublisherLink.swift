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
    private var retryTimerHandle: TimerHandle
    private var needsRetry: Bool
    private var retryPeriod = WallDuration()
    private var nextRetry = SteadyTime()
    private var isDropping: Bool

    init(parent: Subscription, xmlrpcUri: String, transportHints: TransportHints) {
        retryTimerHandle = .none
        needsRetry = false
        isDropping = false
        self.parent = parent
        self.connectionId = 0
        self.publisherXmlrpcUri = xmlrpcUri
        self.transportHints = transportHints
        self.latched = false
    }

    deinit {
        if !retryTimerHandle.isNone {
            getInternalTimerManager().remove(timerHandle: retryTimerHandle)
        }
        connection?.drop(reason: .destructing)
    }

    func initialize(ros: Ros, connection: InboundConnection) {
        self.connection = connection
        connection.initialize(owner: self)

        let header: StringStringMap = ["topic": parent.name,
                                     "md5sum": parent.md5sum,
                                     "callerid": ros.name,
                                     "type": parent.datatype,
                                     "tcp_nodelay": transportHints.getTCPNoDelay() ? "1" : "0"]

        connection.writeHeader(keyVals: header).whenComplete { result in
            switch result {
            case .success:
                ROS_DEBUG("Header written for topic \(self.parent.name)")
            case .failure(let error):
                ROS_ERROR("failed to write header: \(error)")
            }
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
