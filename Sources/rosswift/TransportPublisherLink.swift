//
//  TransportPublisherLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation
import RosTime
import StdMsgs
import Synchronization

final class TransportPublisherLink: PublisherLink, Sendable {
    let connectionId = UUID()
    let parent: Subscription
    let publisherXmlrpcUri: String
    let transportHints: TransportHints

    // Protocol's mutable header fields plus the owning connection reference,
    // kept under a single lock so reads and the multi-field write in setHeader
    // observe a consistent state.
    private struct State: Sendable {
        var latched: Bool = false
        var callerId: String = ""
        var header: Header?
        var md5sum: String = ""
        var connection: InboundConnection?
    }
    private let state = Mutex(State())

    var latched: Bool {
        get { state.withLock { $0.latched } }
        set { state.withLock { $0.latched = newValue } }
    }
    var callerId: String {
        get { state.withLock { $0.callerId } }
        set { state.withLock { $0.callerId = newValue } }
    }
    var header: Header? {
        get { state.withLock { $0.header } }
        set { state.withLock { $0.header = newValue } }
    }
    var md5sum: String {
        get { state.withLock { $0.md5sum } }
        set { state.withLock { $0.md5sum = newValue } }
    }

    init(parent: Subscription, xmlrpcUri: String, transportHints: TransportHints) {
        self.parent = parent
        self.publisherXmlrpcUri = xmlrpcUri
        self.transportHints = transportHints
    }

    deinit {
        if let conn = state.withLock({ $0.connection }) {
            conn.dropConnection(reason: .destructing)
        }
    }

    func initialize(rosName: String, connection: InboundConnection) async {
        state.withLock { $0.connection = connection }
        let headerVals: StringStringMap = ["topic": parent.name,
                                           "md5sum": parent.md5sum.withLock({ $0 }),
                                           "callerid": rosName,
                                           "type": parent.datatype,
                                           "tcp_nodelay": transportHints.getTCPNoDelay() ? "1" : "0"]
        await connection.initialize(owner: self, headerKeyVals: headerVals)
    }

    func dropPublisherLink() {
        parent.remove(publisherLink: self)
        let connection: InboundConnection? = state.withLock {
            let c = $0.connection
            $0.connection = nil
            return c
        }
        connection?.dropConnection(reason: .destructing)
    }

    func getTransportInfo() -> String {
        return state.withLock { $0.connection }?.getTransportInfo() ?? ""
    }
}
