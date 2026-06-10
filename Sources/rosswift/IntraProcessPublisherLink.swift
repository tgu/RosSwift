//
//  IntraProcessPublisherLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-23.
//

import Foundation
import Atomics
import StdMsgs
import Synchronization

final class IntraProcessPublisherLink: PublisherLink, Sendable {
    let parent: Subscription
    let connectionId = UUID()
    let publisherXmlrpcUri: String
    let transportHints: TransportHints

    // Protocol's mutable header fields plus the local subscriber-link reference,
    // kept under a single lock so reads and setHeader's multi-field writes
    // observe a consistent state.
    private struct State: Sendable {
        var latched: Bool = false
        var callerId: String = ""
        var header: Header?
        var md5sum: String = ""
        var publisher: IntraProcessSubscriberLink?
    }
    private let state = Mutex(State())

    let isDropped = ManagedAtomic(false)

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

    func setPublisher(callerId: String, publisher: IntraProcessSubscriberLink) -> Bool {
        state.withLock { $0.publisher = publisher }
        let header = Header(headers: ["callerid": callerId,
                                      "topic": parent.name,
                                      "type": publisher.dataType,
                                      "md5sum": publisher.md5Sum,
                                      "message_definition": publisher.messageDefinition,
                                      "latching": publisher.isLatching ? "1" : "0"
                                     ])
        return setHeader(header: header)
    }

    func dropPublisherLink() {
        guard isDropped.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged else {
            return
        }
        let publisher: IntraProcessSubscriberLink? = state.withLock {
            let p = $0.publisher
            $0.publisher = nil
            return p
        }
        publisher?.dropParentPublication()
        ROS_DEBUG("Connection to local publisher on topic [\(parent.name)] dropped")
        parent.remove(publisherLink: self)
    }

    func handleMessage(m: SerializedMessage) async {
        if isDropped.load(ordering: .relaxed) {
            return
        }
        let connectionHeader = state.withLock { $0.header?.headers ?? StringStringMap() }
        await parent.handle(message: m,
                            connectionHeader: connectionHeader,
                            link: self)
    }
}
