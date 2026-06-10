//
//  SubscriberLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-05.
//

import Foundation
import StdMsgs

protocol SubscriberLink: Sendable {
    var connectionId: UUID { get }
    var destinationCallerId: String { get }
    var parent: Publication! { get }
    var topic: String { get }
    var transportInfo: String { get }
    
    func dropParentPublication()
    func enqueueMessage(_ m: consuming SerializedMessage) async
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
