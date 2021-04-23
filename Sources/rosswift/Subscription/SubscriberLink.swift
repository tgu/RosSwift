//
//  SubscriberLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-05.
//

import StdMsgs

protocol SubscriberLink {
    var connectionId: UInt { get }
    var destinationCallerId: String { get }
    var parent: Publication! { get }
    var topic: String { get }
    var transportInfo: String { get }

    func dropParentPublication()
    func enqueueMessage(m: SerializedMessage)
}

extension SubscriberLink {
    var dataType: String {
        return parent.datatype
    }

    var md5Sum: String {
        return parent.md5sum
    }

    var messageDefinition: String {
        return parent.messageDefinition
    }
}
