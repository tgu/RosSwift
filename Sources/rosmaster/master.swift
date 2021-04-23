//
//  master.swift
//  rosmaster
//
//  Created by Thomas Gustafsson on 2019-04-04.
//

import Foundation
import NIO

#if os(Linux)
import NetService
#endif


public let defaultMasterPort = 11311
let threadGroup = MultiThreadedEventLoopGroup(numberOfThreads: 1)

public class Master: NSObject, NetServiceDelegate {
    public let host: String
    public let port: Int
    let handler: RosMasterHandler
    let masterNode: XMLRPCServer
    let service: NetService


    public init(host: String, port: Int = defaultMasterPort) {
        self.host = host
        self.port = port

        // Start the ROS Master

        handler = RosMasterHandler()
        masterNode = XMLRPCServer(group: threadGroup,
                                       handler: handler.executeMethod(methodName:params:))
        
        
        // advertise our presense with zeroconf (Bonjour)
        
        service = NetService(domain: "local.",
                             type: "_ros._tcp.",
                             name: "rosmaster",
                             port: Int32(port))
        
        super.init()
        
        service.delegate = self
    }
    
    public func start() -> EventLoopFuture<XMLRPCServer> {
        service.publish()
        return self.masterNode.start(host: host, port: port)
    }

    public func stop() -> EventLoopFuture<Void> {
        service.stop()
        return masterNode.stop()
    }

}


