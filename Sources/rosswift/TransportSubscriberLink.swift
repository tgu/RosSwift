//
//  TransportSubscriberLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-07.
//

import Foundation
import NIO
import StdMsgs
import Synchronization

final class TransportSubscriberLink: SubscriberLink {
    let _parent: Mutex<Publication?>
    let connectionId: UUID
    let destinationCallerId: String
    let topic: String
    let connection: Connection
    var transportInfo: String { return connection.transportInfo }
    var parent: Publication! { _parent.withLock({$0})}
    
    init?(ros: Ros, connection: consuming Connection, header: borrowing Header) {
        self.connection = connection
        guard let _topic = header["topic"] else {
            let msg = "Header from subscriber did not have the required element: topic"
            ROS_ERROR(msg)
            connection.sendHeaderError(msg)
            return nil
        }
        
        // This will get validated by validateHeader below
        let clientCallerId = header["callerid"] ?? ""
        guard let pub = ros.topicManager.lookupPublication(topic: _topic) else {
            let msg = "received a connection for a nonexistent topic [\(_topic)] " +
            "from [\(connection.remoteAddress)] [\(clientCallerId)]."
            ROS_ERROR(msg)
            connection.sendHeaderError(msg)
            return nil
        }
        
        var errorMsg = ""
        if !pub.validateHeader(header: header, errorMsg: &errorMsg) {
            ROS_ERROR(errorMsg)
            connection.sendHeaderError(errorMsg)
            return nil
        }
        
        destinationCallerId = clientCallerId
        connectionId = UUID()
        topic = pub.name
        _parent = Mutex(pub)
        
        // Send back a success, with info
        var m = StringStringMap()
        m["type"] = pub.datatype
        m["md5sum"] = pub.md5sum
        m["message_definition"] = pub.messageDefinition
        m["callerid"] = ros.name
        m["latching"] = pub.isLatched ? "1" : "0"
        m["topic"] = topic
        
        pub.addSubscriberLinkSync(self)
        
        connection.writeHeader(keyVals: m).whenComplete { result in
            switch result {
            case .success:
                break
            case .failure(let error):
                ROS_ERROR("Failed to write header: \(error)")
                // FIXME: is this correct?
            }
        }
    }
    
    func dropParentPublication() {
        parent?.removeSubscriberLink(self)
    }
    
    func enqueueMessage(_ m: SerializedMessage) {
        connection.write(buffer: m.buf).whenFailure({ error in
            ROS_ERROR("writing \(m), \(error)")
        })
    }
    
}
