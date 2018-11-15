//
//  ServiceClientLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-07.
//

import Foundation

final class ServiceClientLink {
    var connection: nio.Connection? = nil
    var parent: ServiceProtocol? = nil
    var persistent = false

    func drop() {
        assert(connection != nil)
        assert(parent != nil)
        parent?.removeServiceClientLink(self)
    }

    func initialize(connection: nio.Connection) {
        self.connection = connection
    }

    func handleHeader(header: Header) -> Bool {
        guard let md5sum = header.getValue(key: "md5sum"), let service = header.getValue(key: "service"), let client_callerid = header.getValue(key: "callerid") else {
            let msg = "bogus tcpros header. did not have the required elements: md5sum, service, callerid"
            ROS_LOG_ERROR(msg)
            connection?.sendHeaderError(msg)
            return false
        }
        if let persist = header.getValue(key: "persistent") {
            persistent = persist == "1" || persist == "true"
        }

        ROS_DEBUG("Service client [\(client_callerid)] wants service [\(service)] with md5sum [\(md5sum)]")

        guard let ss = ServiceManager.instance.lookupServicePublication(service: service) else {
            let msg = "received a tcpros connection for a nonexistent service [\(service)]."
            ROS_LOG_ERROR(msg)
            connection?.sendHeaderError(msg)
            return false
        }

        if ss.md5sum != md5sum && md5sum != "*" && ss.md5sum != "*" {
            let msg = "client wants service \(service) to have md5sum \(md5sum) but it has \(ss.md5sum). Dropping connection."
            ROS_LOG_ERROR(msg)
            connection?.sendHeaderError(msg)
            return false
        }

        // Check whether the service (ss here) has been deleted from
        // advertised_topics through a call to unadvertise(), which could
        // have happened while we were waiting for the subscriber to
        // provide the md5sum.

        if ss.isDropped() {
            let msg = "received a tcpros connection for a nonexistent service \(service)"
            ROS_LOG_ERROR(msg)
            connection?.sendHeaderError(msg)
            return false
        }

        self.parent = ss
        let m : M_string = ["request_type": ss.request_data_type,
                            "response_type": ss.response_data_type,
                            "type": ss.data_type,
                            "md5sum": ss.md5sum,
                            "callerid": Ros.this_node.getName()]

//        ROS_DEBUG("\(#file):\(#line)  write from \(self.connection?.channel.localAddress) => \(self.connection?.channel.remoteAddress)")
//        ROS_DEBUG(m)

        connection?.writeHeader(key_vals: m).map {
            ROS_DEBUG("data written")
        }.whenFailure { (error) in
            ROS_DEBUG("Could not wite header: \(m)\n\n error: \(error)")
        }

//        connection?.channel.writeAndFlush(m).whenFailure({ (error) in
//            ROS_DEBUG("Could not wite data: \(m)\n\n error: \(error)")
//        })
        //connection?.writeHeader(key_vals: m)
        ss.addServiceClientLink(link: self)

        return true

    }



}
