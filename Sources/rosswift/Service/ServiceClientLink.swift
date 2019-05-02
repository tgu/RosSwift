//
//  ServiceClientLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-07.
//

import Foundation

internal final class ServiceClientLink {
    var connection: Connection?
    var parent: ServiceProtocol?
    var persistent = false

    func drop() {
        assert(connection != nil)
        assert(parent != nil)
        parent?.removeServiceClientLink(self)
    }

    func initialize(connection: Connection) {
        self.connection = connection
    }

    func handleHeader(header: Header, ros: Ros) -> Bool {
        guard let md5sum = header.getValue(key: "md5sum"),
            let service = header.getValue(key: "service"),
            let clientCallerId = header.getValue(key: "callerid") else {

            let msg = "bogus tcpros header. did not have the required elements: md5sum, service, callerid"
            ROS_LOG_ERROR(msg)
            connection?.sendHeaderError(msg)
            return false
        }
        if let persist = header.getValue(key: "persistent") {
            persistent = persist == "1" || persist == "true"
        }

        ROS_DEBUG("Service client [\(clientCallerId)] wants service [\(service)] with md5sum [\(md5sum)]")

        guard let servicePublication = ros.serviceManager.lookupServicePublication(service: service) else {
            let msg = "received a tcpros connection for a nonexistent service [\(service)]."
            ROS_LOG_ERROR(msg)
            connection?.sendHeaderError(msg)
            return false
        }

        if servicePublication.md5sum != md5sum && md5sum != "*" && servicePublication.md5sum != "*" {
            let msg = "client wants service \(service) to have md5sum \(md5sum)" +
                      " but it has \(servicePublication.md5sum). Dropping connection."
            ROS_LOG_ERROR(msg)
            connection?.sendHeaderError(msg)
            return false
        }

        // Check whether the service (ss here) has been deleted from
        // advertised_topics through a call to unadvertise(), which could
        // have happened while we were waiting for the subscriber to
        // provide the md5sum.

        if servicePublication.isDropped {
            let msg = "received a tcpros connection for a nonexistent service \(service)"
            ROS_LOG_ERROR(msg)
            connection?.sendHeaderError(msg)
            return false
        }

        self.parent = servicePublication
        let m: StringStringMap = ["request_type": servicePublication.requestDataType,
                            "response_type": servicePublication.responseDataType,
                            "type": servicePublication.dataType,
                            "md5sum": servicePublication.md5sum,
                            "callerid": ros.name]

        connection?.writeHeader(keyVals: m).map {
            ROS_DEBUG("data written")
        }.whenFailure { error in
            ROS_DEBUG("Could not wite header: \(m)\n\n error: \(error)")
        }
        servicePublication.addServiceClientLink(link: self)

        return true

    }

}
