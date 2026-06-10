//
//  master.swift
//  rosmaster
//
//  Created by Thomas Gustafsson on 2019-04-04.
//

import Foundation
import NIO

#if os(Linux) || os(watchOS)
//import NetService
protocol NetServiceDelegate {}
#endif


public let defaultMasterPort = 11311
let threadGroup = MultiThreadedEventLoopGroup(numberOfThreads: 1)

public final class Master: NSObject, NetServiceDelegate, @unchecked Sendable {
    public let host: String
    /// The port passed at init. When 0, `start()` lets the OS pick a port;
    /// the actually-bound port is then available via `boundPort` / `port`.
    public let requestedPort: Int
    let handler: RosMasterHandler
    let masterNode: XMLRPCServer
    #if !(os(Linux) || os(watchOS))
    let netService: NetService?
    #endif

    /// The port currently in use. Equals `requestedPort` if non-zero and
    /// before `start()`; falls through to the actually-bound port after
    /// `start()` succeeds (which is necessary when `requestedPort == 0`).
    public var port: Int {
        let bound = masterNode.boundPort
        return bound != 0 ? bound : requestedPort
    }

    public var address: String {
        "http://\(host):\(port)"
    }

    public init(host: String, port: Int = defaultMasterPort, advertise: Bool = true) {
        self.host = host
        self.requestedPort = port

        // Start the ROS Master

        handler = RosMasterHandler()
        let h = handler
        masterNode = XMLRPCServer(group: threadGroup,
                                  handler: { method, params in
                                      await h.executeMethod(methodName: method, params: params)
                                  })
        
        
        // advertise our presense with zeroconf (Bonjour)
        
        #if !(os(Linux)  || os(watchOS))
        if advertise {
        netService = NetService(domain: "local.",
                             type: "_ros._tcp.",
                             name: "rosmaster",
                             port: Int32(port))
        } else {
            netService = nil
        }
        #endif

        super.init()
        
        #if !(os(Linux) || os(watchOS))
        if advertise {
        netService?.delegate = self
        }
        #endif
    }
    
    public func start() -> EventLoopFuture<XMLRPCServer> {
        #if !(os(Linux) || os(watchOS))
        netService?.publish()
        #endif
        return self.masterNode.start(host: host, port: requestedPort)
    }

    public func stop() -> EventLoopFuture<Void> {
        #if !(os(Linux) || os(watchOS))
        netService?.stop()
        #endif
        return masterNode.stop()
    }

}


