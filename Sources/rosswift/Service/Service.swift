//
//  Service.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-11.
//

import NIO
import RosTime
import StdMsgs
import BinaryCoder

/// Namespace for one-shot service calls and service-availability helpers.
///
/// For repeated calls, prefer a persistent `ServiceClient` from
/// `NodeHandle.serviceClient`.
public enum Service {

    /// Invoke an RPC service.
    ///
    /// This method invokes an RPC service on a remote server,
    /// looking up the service location first via the master.
    ///
    /// - Parameters:
    ///     - serviceName: The name of the service.
    ///     - req: The request message
    /// - Returns: A future with the response
    
    
    public static func call<MReq: ServiceMessage, MRes: ServiceMessage>(node: NodeHandle, serviceName: String, req: MReq) async throws -> MRes {

        // name is resolved in serviceClient

        let client = await node.serviceClient(service: serviceName, md5sum: MReq.srvMd5sum)
        return try await client.call(req: req)
    }

    static func call<R: ServiceRequestMessage>(node: NodeHandle, name: String, request: R) async -> R.ServiceType.Response? {
        return try? await call(node: node, serviceName: name, req: request)
    }

    static func call<MReq: ServiceMessage, MRes: ServiceMessage>(node: NodeHandle, serviceName: String, req: MReq, response: inout MRes) async  -> Bool {
        do {
            let resp: MRes = try await call(node: node, serviceName: serviceName, req: req)
            response = resp
            return true
        } catch {
            return false
        }
    }
    
    /// Wait for a service to be advertised and available.  Blocks until it is.
    ///
    /// - Parameters:
    ///   - serviceName: Name of the service to wait for
    ///   - timeout: The amount of time to wait for, in milliseconds.  If timeout is -1, waits until the node is shutdown
    /// - Returns: true on success, false otherwise
    public static func waitForService(ros: Ros, serviceName: String, timeout milliseconds: Int32) async -> Bool {
        await waitForService(ros: ros, serviceName: serviceName, timeout: .milliseconds(milliseconds) )
    }
    
    /// Wait for a service to be advertised and available.  Blocks until it is.
    ///
    /// - Parameter serviceName: Name of the service to wait for.
    /// - Parameter timeout: The amount of time to wait for before timing out.  If timeout is -1 (default),
    /// waits until the node is shutdown
    /// - Returns: true on success, false otherwise
    public static func waitForService(ros: Ros, serviceName: String, timeout: RosDuration = RosDuration()) async -> Bool {
        let mappedNames = ros.resolve(name: serviceName)
        let startTime = Time.now
        var printed = false
        var result = false
        while ros.isRunning.load(ordering: .relaxed) {
            if await exists(ros: ros, serviceName: serviceName, printFailureReason: !printed) {
                result = true
                break
            } else {
                printed = true
                if !timeout.isZero() {
                    let currentTime = Time.now
                    if currentTime - startTime >= timeout {
                        return false
                    }
                }
                
                await RosDuration(milliseconds: 20).sleep()
            }
        }
        
        if printed && ros.isRunning.load(ordering: .relaxed) {
            ROS_DEBUG("waitForService: Service [\(String(describing: mappedNames))] is now available.")
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
    
    public static func exists(ros: Ros, serviceName: String, printFailureReason: Bool) async -> Bool {
        guard let mappedName = ros.resolve(name: serviceName) else {
            return false
        }
        
        if let server = await ros.serviceManager.lookupService(name: mappedName) {
            let keymap = ["probe": "1", "md5sum": "*", "callerid": ros.name, "service": mappedName]
            let transport = TransportTCP()
            do {
                
                try await transport.connect(host: server.host, port: Int(server.port)).map { channel -> Void in
                    _ = channel.eventLoop.makeCompletedFuture {
                        
                        _ = try channel.pipeline.syncOperations.addHandlers([ByteToMessageHandler(MessageDelimiterCodec()),
                                                                             ByteToMessageHandler(HeaderMessageCodec()),
                                                                             TransportTCP.Handler(callback: callback)])
                    }
                    let buffer = Header.write(keyVals: keymap)
                    do {
                        let sizeBuffer = try BinaryEncoder.encode(UInt32(buffer.count))
                        var buf = channel.allocator.buffer(capacity: buffer.count + 4)
                        buf.writeBytes(sizeBuffer + buffer)
                        let data = IOData.byteBuffer(buf)
                        
                        channel.writeAndFlush(data).whenFailure { error in
                            ROS_DEBUG("exists, write failed to \(String(describing: channel.remoteAddress))\nerror: \(error))")
                        }
                    } catch {
                        ROS_ERROR("encode failed: \(error)")
                    }
                }.get()
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
