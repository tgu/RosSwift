//
//  SingleSubscriberPublisher.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import BinaryCoder
import Foundation
import StdMsgs

public final class SingleSubscriberPublisher {
    var link: SubscriberLink
    public var callerId: String {
        return link.destinationCallerId
    }

    init(link: SubscriberLink) {
        self.link = link
    }

    public func publish<M: Message>(_ msg: M) {
        let ser = SerializedMessage(msg: msg)
        publish(m: ser)
    }
    
    func publish(m: StdMsgs.SerializedMessage) {
        link.enqueueMessage(m: m)
    }

}
