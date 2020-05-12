//
//  SingleSubscriberPublisher.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import StdMsgs

public struct SingleSubscriberPublisher {
    var link: SubscriberLink

    public var callerId: String {
        return link.destinationCallerId
    }

    public func publish<M: Message>(_ msg: M) {
        let ser = SerializedMessage(msg: msg)
        link.enqueueMessage(m: ser)
    }
}
