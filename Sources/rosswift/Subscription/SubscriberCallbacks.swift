//
//  SubscriberCallbacks.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation

public typealias SubscriberStatusCallback = (SingleSubscriberPublisher) -> Void

final class SubscriberCallbacks {
    let connect: SubscriberStatusCallback?
    let disconnect: SubscriberStatusCallback?
    let hasTrackedObject: Bool
    let trackedObject: AnyObject?

    init(connect: SubscriberStatusCallback?,
         disconnect: SubscriberStatusCallback?,
         hasTrackedObject: Bool,
         trackedObject: AnyObject? = nil) {

        self.connect = connect
        self.disconnect = disconnect
        self.hasTrackedObject = hasTrackedObject
        self.trackedObject = trackedObject
    }
}
