//
//  MessageEvent.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation
import StdMsgs
import RosTime


public final class MessageEvent<M: Message> {
    typealias CreateFunction = () -> M

    let message_: M
    let connection_header_ : M_string
    let receipt_time_ : RosTime.Time
    let nonconst_need_copy_ : Bool
    let create_ : CreateFunction

    init(message: M, connection_header: M_string, receipt_time: RosTime.Time, nonconst_need_copy: Bool, create: @escaping CreateFunction)
    {
        self.message_ = message;
        self.connection_header_ = connection_header;
        self.receipt_time_ = receipt_time;
        self.nonconst_need_copy_ = nonconst_need_copy;
        self.create_ = create;
    }


    convenience init(event: MessageEvent<M>, create: @escaping CreateFunction) {
        self.init(message: event.message_, connection_header: event.connection_header_, receipt_time: event.receipt_time_, nonconst_need_copy: event.nonconst_need_copy_, create: create)
    }


    
    deinit {
        ROS_DEBUG("deinit()")
    }
}
