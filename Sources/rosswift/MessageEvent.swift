//
//  MessageEvent.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation
import RosTime
import StdMsgs

public final class MessageEvent<M: Message> {
    typealias CreateFunction = () -> M

    let message: M
    let connectionHeader: StringStringMap
    let receiptTime: RosTime.Time
    let nonconstNeedCopy: Bool
    let create: CreateFunction

    init(message: M,
         header: StringStringMap,
         receiptTime: RosTime.Time,
         nonconstNeedCopy: Bool,
         create: @escaping CreateFunction) {

        self.message = message
        self.connectionHeader = header
        self.receiptTime = receiptTime
        self.nonconstNeedCopy = nonconstNeedCopy
        self.create = create
    }

    convenience init(event: MessageEvent<M>, create: @escaping CreateFunction) {
        self.init(message: event.message,
                  header: event.connectionHeader,
                  receiptTime: event.receiptTime,
                  nonconstNeedCopy: event.nonconstNeedCopy,
                  create: create)
    }

    deinit {
        ROS_DEBUG("deinit()")
    }
}
