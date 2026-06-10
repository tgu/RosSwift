//
//  IntraProcessSubscriberLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-23.
//

import Foundation
import Atomics
import StdMsgs

struct IntraProcessSubscriberLink: SubscriberLink, Sendable {
    weak var parent: Publication!
    let connectionId: UUID
    let destinationCallerId: String
    let topic: String
    let transportInfo = "INTRAPROCESS"
    
    weak var subscriber: IntraProcessPublisherLink?
    let isDropped = ManagedAtomic(false)
    var isLatching: Bool { parent?.isLatched ?? false }
    
    
    
    init(desitnationName: String, parent: Publication, subscriber: IntraProcessPublisherLink) {
        self.parent = parent
        topic = parent.name
        self.subscriber = subscriber
        connectionId = UUID()
        destinationCallerId = desitnationName
    }
    
    func enqueueMessage(_ m: SerializedMessage) async {
        if isDropped.load(ordering: .relaxed) {
            return
        }

        // We have to remove the four first bytes with length information.
        // `SerializedMessage` is now immutable (buf is `let`), so wrap the
        // stripped bytes in a fresh value.
        let stripped = SerializedMessage(buffer: [UInt8](m.buf.dropFirst(4)))

        await subscriber?.handleMessage(m: stripped)
    }
    
    func dropParentPublication() {
        if isDropped.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged {
            subscriber?.dropPublisherLink()
            // subscriber = nil
            
            ROS_DEBUG("Connection to local subscriber on topic [\(topic)] dropped")
            parent.removeSubscriberLink(self)
        }
    }
    
    
    
}
