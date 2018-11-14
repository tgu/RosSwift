//
//  TransportSubscriberLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-07.
//

import Foundation
import StdMsgs
import NIO

let dropNotification = Notification.Name(rawValue: "dropConnection")



extension Ros {
    final class TransportSubscriberLink: SubscriberLink {

    weak var parent : Publication!
    var connection_id : UInt = 0
    var destination_caller_id  : String = ""
    var topic : String = ""

    var header_written = false
    var connection : nio.Connection?
    var running = true

    func isIntraprocess() -> Bool {
        return false
    }

    
    init(connection: nio.Connection)  {
        self.connection = connection

        #if os(OSX)
        NotificationCenter.default.addObserver(self, selector: #selector(onConnectionDropped(_:)), name: dropNotification, object: connection)
        #endif
    
    }

    func drop() {
        if let conn = connection {
            if conn.sending_header_error {
                NotificationCenter.default.removeObserver(self)
            } else {
                parent?.removeSubscriberLink(self)

                connection = nil
            }
        }
    }

    func handleHeader(header: Header) -> Bool
    {
        guard let connection = connection else {
            ROS_ERROR("no connection")
            return false
        }
        guard let topic = header.getValue(key: "topic") else {
            let msg = "Header from subscriber did not have the required element: topic"
            ROS_ERROR(msg)
            connection.sendHeaderError(msg)
            return false
        }

        // This will get validated by validateHeader below
        let client_callerid = header.getValue(key: "callerid") ?? ""
        guard let pt = TopicManager.instance.lookupPublication(topic: topic) else {
            let msg = "received a connection for a nonexistent topic [\(topic)] from [\(connection.remoteAddress)] [\(client_callerid)]."
            ROS_ERROR(msg)
            connection.sendHeaderError(msg)
            return false
        }

        var error_msg = ""
        if !pt.validateHeader(header: header, error_msg: &error_msg) {
            ROS_ERROR(error_msg)
            connection.sendHeaderError(error_msg)
            return false
        }

        destination_caller_id = client_callerid
        connection_id = UInt(Ros.ConnectionManager.instance.getNewConnectionID())
        self.topic = pt.name
        parent = pt

        // Send back a success, with info
        var m = M_string()
        m["type"] = pt.datatype
        m["md5sum"] = pt.md5sum
        m["message_definition"] = pt.message_definition
        m["callerid"] = Ros.this_node.getName()
        m["latching"] = pt.isLatching() ? "1" : "0"
        m["topic"] = topic

        pt.addSubscriberLink(self)

        connection.writeHeader(key_vals: m).whenComplete {
            self.header_written = true
        }

        return true
    }

        #if os(OSX)
    @objc func onConnectionDropped(_ note: Notification) {
        running = false
        ROS_INFO("Connection to subscriber [\(self.connection?.remoteString ?? "#####")] to topic [\(self.topic)] dropped")
        parent?.removeSubscriberLink(self)
    }

        #endif

    func enqueueMessage(m: SerializedMessage, ser: Bool, nocopy: Bool) {
        if !ser {
            return
        }

        connection?.write(buffer: m.buf).whenFailure({ (error) in
            ROS_ERROR("writing \(m), \(error)")
        })

    }

    }



}

