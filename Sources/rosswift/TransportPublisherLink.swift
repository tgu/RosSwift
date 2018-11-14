//
//  TransportPublisherLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation
import StdMsgs
import RosTime

final class TransportPublisherLink: PublisherLink {
    private var connection_ : InboundConnection? = nil
    private var retry_timer_handle_ : Int32
    private var needs_retry_ : Bool
    private var retry_period_ = RosTime.WallDuration()
    private var next_retry_ = RosTime.SteadyTime()
    private var dropping_ : Bool

    override init(parent: Subscription, xmlrpcUri: String, transportHints: TransportHints) {
        retry_timer_handle_ = -1
        needs_retry_ = false
        dropping_ = false
        super.init(parent: parent, xmlrpcUri: xmlrpcUri, transportHints: transportHints)
    }

    deinit {
        if retry_timer_handle_ != -1 {
            getInternalTimerManager().remove(timer_handle: retry_timer_handle_)
        }
        connection_?.drop(reason: .Destructing)
    }



    func initialize(connection: InboundConnection) {
        connection_ = connection
        connection.initialize(owner: self)

        let header : M_string = ["topic": parent.name,
                                     "md5sum": parent.md5sum_,
                                     "callerid": Ros.this_node.getName(),
                                     "type": parent.datatype,
                                     "tcp_nodelay": transportHints.getTCPNoDelay() ? "1" : "0"]


        connection.writeHeader(key_vals: header).whenComplete {
            ROS_DEBUG("TransportPublisherLink: header is written")
        }
    }

    override func drop() {
        dropping_ = true
        parent.remove(publisherLink: self)
        connection_?.drop(reason: .Destructing)
        connection_ = nil
    }

    override func getTransportInfo() -> String {
        return connection_?.getTransportInfo() ?? ""
    }


}
