//
//  SubscriberLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-05.
//

import Foundation
import StdMsgs

protocol SubscriberLink: class {
    var parent: Publication! { get }
    var connectionId: UInt { get }
    var destinationCallerId: String { get }
    var topic: String { get }

    func drop()
    func isIntraprocess() -> Bool
    func enqueueMessage(m: SerializedMessage)
    func getTransportInfo() -> String
}

extension SubscriberLink {
    func getDataType() -> String {
        return parent.datatype
    }

    func getMD5Sum() -> String {
        return parent.md5sum
    }

    func getMessageDefinition() -> String {
        return parent.messageDefinition
    }

    func getPublishTypes(ser: inout Bool, nocopy: inout Bool, ti: TypeInfo) {
        ser = true
        nocopy = false
    }
}
