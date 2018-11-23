//
//  TransportSubscriberLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-07.
//

import Foundation
import NIO
import StdMsgs

extension Ros {
    final class TransportSubscriberLink: SubscriberLink {

    weak var parent: Publication!
    var connectionId: UInt = 0
    var destinationCallerId: String = ""
    var topic: String = ""

    var headerWritten = false
    var connection: Nio.Connection?
    var running = true

    func isIntraprocess() -> Bool {
        return false
    }

    init(connection: Nio.Connection) {
        self.connection = connection
    }

    func drop() {
        if let conn = connection {
            if conn.isSendingHeaderError {
                NotificationCenter.default.removeObserver(self)
            } else {
                parent?.removeSubscriberLink(self)

                connection = nil
            }
        }
    }

    func getTransportInfo() -> String {
        return connection?.getTransportInfo() ?? ""
    }

    func handleHeader(header: Header) -> Bool {
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
        let clientCallerId = header.getValue(key: "callerid") ?? ""
        guard let pub = TopicManager.instance.lookupPublication(topic: topic) else {
            let msg = "received a connection for a nonexistent topic [\(topic)] " +
                      "from [\(connection.remoteAddress)] [\(clientCallerId)]."
            ROS_ERROR(msg)
            connection.sendHeaderError(msg)
            return false
        }

        var errorMsg = ""
        if !pub.validateHeader(header: header, errorMsg: &errorMsg) {
            ROS_ERROR(errorMsg)
            connection.sendHeaderError(errorMsg)
            return false
        }

        destinationCallerId = clientCallerId
        connectionId = UInt(Ros.ConnectionManager.instance.getNewConnectionID())
        self.topic = pub.name
        parent = pub

        // Send back a success, with info
        var m = StringStringMap()
        m["type"] = pub.datatype
        m["md5sum"] = pub.md5sum
        m["message_definition"] = pub.messageDefinition
        m["callerid"] = Ros.ThisNode.getName()
        m["latching"] = pub.isLatching() ? "1" : "0"
        m["topic"] = topic

        pub.addSubscriberLink(self)

        connection.writeHeader(keyVals: m).whenComplete {
            self.headerWritten = true
        }

        return true
    }

    func enqueueMessage(m: SerializedMessage) {
        connection?.write(buffer: m.buf).whenFailure({ error in
            ROS_ERROR("writing \(m), \(error)")
        })
    }

    }

}
