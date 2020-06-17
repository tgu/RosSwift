//
//  TransportSubscriberLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-07.
//

import Foundation
import NIO
import StdMsgs

final class TransportSubscriberLink: SubscriberLink {
    
    weak var parent: Publication!
    var connectionId: UInt = 0
    var destinationCallerId: String = ""
    var topic: String = ""
    var connection: Connection?
    let isIntraprocess = false

    init(connection: Connection) {
        self.connection = connection
    }
    
    func dropPublication() {
        if let conn = connection, !conn.isSendingHeaderError {
            parent?.removeSubscriberLink(self)
            connection = nil
        }
    }
    
    var transportInfo: String { return connection?.getTransportInfo() ?? "" }
    
    func handleHeader(ros: Ros, header: Header) -> Bool {
        guard let connection = connection else {
            ROS_ERROR("no connection")
            return false
        }
        guard let topic = header["topic"] else {
            let msg = "Header from subscriber did not have the required element: topic"
            ROS_ERROR(msg)
            connection.sendHeaderError(msg)
            return false
        }
        
        // This will get validated by validateHeader below
        let clientCallerId = header["callerid"] ?? ""
        guard let pub = ros.topicManager.lookupPublication(topic: topic) else {
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
        connectionId = UInt(ros.connectionManager.getNewConnectionID())
        self.topic = pub.name
        parent = pub
        
        // Send back a success, with info
        var m = StringStringMap()
        m["type"] = pub.datatype
        m["md5sum"] = pub.md5sum
        m["message_definition"] = pub.messageDefinition
        m["callerid"] = ros.name
        m["latching"] = pub.isLatching() ? "1" : "0"
        m["topic"] = topic
        
        pub.addSubscriberLink(self)
        
        connection.writeHeader(keyVals: m).whenComplete { result in
            switch result {
            case .success:
                break
            case .failure(let error):
                ROS_ERROR("Failed to write header: \(error)")
                // FIXME: is this correct?
            }
        }
        
        return true
    }
    
    func enqueueMessage(m: SerializedMessage) {
        connection?.write(buffer: m.buf).whenFailure({ error in
            ROS_ERROR("writing \(m), \(error)")
        })
    }
    
}
