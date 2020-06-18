//
//  master.swift
//  rosmaster
//
//  Created by Thomas Gustafsson on 2019-04-04.
//

import Foundation
import NIO

public let defaultMasterPort = 11311
let threadGroup = MultiThreadedEventLoopGroup(numberOfThreads: 1)

public class Master {
    let host: String
    let port: Int
    let handler: RosMasterHandler
    let masterNode: XMLRPCServer


    public init(host: String, port: Int = defaultMasterPort) {
        self.host = host
        self.port = port

        // Start the ROS Master

        handler = RosMasterHandler()
        masterNode = XMLRPCServer(group: threadGroup,
                                       handler: handler.executeMethod(methodName:params:))
    }

    public func start() -> EventLoopFuture<XMLRPCServer> {
        return self.masterNode.start(host: host, port: port)
    }

    public func stop() -> EventLoopFuture<Void> {
        return masterNode.stop()
    }

}


