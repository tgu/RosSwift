//
//  Service.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-11.
//

import Foundation
import NIO
import RosTime
import StdMsgs

public struct Service {

    public static func call<MReq: ServiceMessage, MRes: ServiceMessage>(serviceName: String, req: MReq) -> EventLoopFuture<MRes?> {
        let node = Ros.NodeHandle()
        let client = node.serviceClient(service: Ros.Names.resolve(name: serviceName), md5sum: MReq.srvMd5sum)
        return client.call(req: req)
    }

    static func call<Service: ServiceProt>(name: String, service: inout Service) -> Bool {
        return call(serviceName: name, req: service.request, response: &service.response)
    }

    static func call<MReq: ServiceMessage, MRes: ServiceMessage>(serviceName: String, req: MReq, response: inout MRes) -> Bool {
        let res: EventLoopFuture<MRes?> = call(serviceName: serviceName, req: req)
        do {
            if let resp = try res.wait() {
                response = resp
                return true
            }
        } catch {
            return false
        }
        return false

    }

    /// Wait for a service to be advertised and available.  Blocks until it is.
    ///
    /// - Parameters:
    ///   - serviceName: Name of the service to wait for
    ///   - timeout: The amount of time to wait for, in milliseconds.  If timeout is -1, waits until the node is shutdown
    /// - Returns: true on success, false otherwise
    static func waitForService(serviceName: String, timeout: Int32) -> Bool {
        let dur = RosTime.Duration(milliseconds: timeout)
        return waitForService(serviceName: serviceName, timeout: dur )
    }

    /// Wait for a service to be advertised and available.  Blocks until it is.
    ///
    /// - Parameter serviceName: Name of the service to wait for.
    /// - Parameter timeout: The amount of time to wait for before timing out.  If timeout is -1 (default),
    /// waits until the node is shutdown
    /// - Returns: true on success, false otherwise
    static func waitForService(serviceName: String, timeout: RosTime.Duration = RosTime.Duration(seconds: TimeInterval(-1))) -> Bool {
        let mappedNames = Ros.Names.resolve(name: serviceName)
        let startTime = RosTime.Time.now()
        var printed = false
        var result = false
        while Ros.isRunning {
            if exists(serviceName: serviceName, printFailureReason: !printed) {
                result = true
                break
            } else {
                printed = true
                if timeout >= RosTime.Duration(seconds: 0) {
                    let currentTime = RosTime.Time.now()
                    if currentTime - startTime >= timeout {
                        return false
                    }
                }

                RosTime.Duration(milliseconds: 20).sleep()
            }
        }

        if printed && Ros.isRunning {
            ROS_DEBUG("waitForService: Service [\(mappedNames)] is now available.")
        }

        return result
    }

    static func callback(m: StringStringMap) {
        ROS_DEBUG(m.debugDescription)
    }

    /// Checks if a service is both advertised and available.
    ///
    /// - Parameter serviceName: Name of the service to check for
    /// - Parameter printFailureReason: Whether to print the reason for failure to the console (service not advertised vs.
    /// could not connect to the advertised host)
    /// - Returns: true if the service is up and available, false otherwise

    static func exists(serviceName: String, printFailureReason: Bool) -> Bool {
        let mappedName = Ros.Names.resolve(name: serviceName)
        var host = ""
        var port: UInt16 = 0
        if ServiceManager.instance.lookupService(name: mappedName, servHost: &host, servPort: &port) {
            let keymap = ["probe": "1", "md5sum": "*", "callerid": Ros.ThisNode.getName(), "service": mappedName]
            let transport = Nio.TransportTCP(pipeline: [Nio.MessageDelimiterCodec(),
                                                        Nio.HeaderMessageCodec(),
                                                        Nio.TransportTCP.Handler(callback: callback)])
            do {

            try transport.connect(host: host, port: Int(port)).map { channel -> Void in
                _ = channel.writeAndFlush(keymap)
            }.wait()
            return true
            } catch {
                ROS_ERROR("\(error)")
                return false
            }

        } else {
            if printFailureReason {
                ROS_DEBUG("waitForService: Service [\(mappedName)] has not been advertised, waiting...")
            }
        }
        return false
    }

}
