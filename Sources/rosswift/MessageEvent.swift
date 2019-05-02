//
//  MessageEvent.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation
import RosTime
import StdMsgs

public struct MessageEvent<M: Message> {
    public let message: M
    public let connectionHeader: StringStringMap
    public let receiptTime: Time

    init(message: M,
         header: StringStringMap,
         receiptTime: Time) {

        self.message = message
        self.connectionHeader = header
        self.receiptTime = receiptTime
    }

}
