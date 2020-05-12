//
//  Servie+Extension.swift
//  
//
//  Created by Thomas Gustafsson on 2020-05-12.
//

import StdMsgs

extension ServiceProt {

    /// Advertise a service.
    ///
    /// This call connects to the master to publicize that the node will be offering
    /// an RPC service with the given name.
    ///
    /// # Example Usage
    ///
    ///     let srv2 = TestStringString.advertise(service: "echo", node: node) {
    ///         let response = $0.data.uppercased()
    ///         return .init(response)
    ///     }
    ///
    /// - Parameters:
    ///     - service: name of service
    ///     - srvFunc: Completion to be called when service is called
    ///


    public static func advertise(service: String, node: NodeHandle, handler: @escaping (Request) -> Response) -> ServiceServer? {
        return node.advertise(service: service, srvFunc: handler)
    }


    /// Invoke an RPC service.
    ///
    /// This method invokes an RPC service on a remote server,
    /// looking up the service location first via the master.
    ///
    /// - Parameters:
    ///     - name: The name of the service.
    ///     - node: The calling node
    ///     - req: The request message
    /// - Returns: An optional response


    public static func call(name: String, node: NodeHandle, req: Request) -> Response? {
        return try? Service.call(node: node, serviceName: name, req: req).wait()
    }
}

extension ServiceRequestMessage {
    public func call(name: String, node: NodeHandle) -> ServiceType.Response? {
        return try? Service.call(node: node, serviceName: name, req: self).wait()
    }
}

