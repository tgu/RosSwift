//
//  SubscriberCallbacks.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation

public typealias SubscriberStatusCallback = (SingleSubscriberPublisher) -> Void


final class SubscriberCallbacks {
    var connect: SubscriberStatusCallback? = nil
    var disconnect: SubscriberStatusCallback? = nil
    var has_tracked_object: Bool = false
    var tracked_object: AnyObject? = nil

    init(connect: SubscriberStatusCallback?, disconnect: SubscriberStatusCallback?, has_tracked_object: Bool, tracked_object: AnyObject? = nil) {
        self.connect = connect
        self.disconnect = disconnect
        self.has_tracked_object = has_tracked_object
        self.tracked_object = tracked_object
    }
}
