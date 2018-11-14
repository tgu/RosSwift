//
//  ServiceManager.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-05.
//

import Foundation
import NIO
import NIOConcurrencyHelpers
import StdMsgs

    final class ServiceManager {
        static let instance = ServiceManager()

        var shutting_down = Atomic<Bool>(value: false)
        var service_publications_ = SynchronizedArray<ServiceProtocol>()
        var service_server_links_ = SynchronizedArray<ServiceServerLink>()

        var connection_manager_ : Ros.ConnectionManager { return Ros.ConnectionManager.instance }
        var xmlrpc_manager_ : XMLRPCManager { return XMLRPCManager.instance }

        private init() {
        }

        deinit {
            shutdown()
        }

        func start() {
            shutting_down.store(false)
            ROS_DEBUG("servicemanager start")
        }

        func shutdown() {
            guard shutting_down.compareAndExchange(expected: false, desired: true) else {
                return
            }

            ROS_DEBUG("ServiceManager::shutdown(): unregistering our advertised services")

            let localpub = service_publications_.all()
            service_publications_.removeAll()

            localpub.forEach {
                ROS_DEBUG("shutdown service \($0.name)")
                self.unregisterService(service: $0.name)
                $0.drop()
            }
            
            service_server_links_.removeAll { (links) in
                links.forEach {
                    $0.channel?.close()
                }
            }
        }

        func advertiseService<MReq: ServiceMessage, MRes: ServiceMessage>(_ ops: AdvertiseServiceOptions<MReq, MRes>) -> Bool {
            if shutting_down.load() {
                return false
            }

            if isServiceAdvertised(serv_name: ops.service) {
                ROS_ERROR("Tried to advertise a service that is already advertised in this node [\(ops.service)]")
                return false
            }

            let pub = ServicePublication(name: ops.service, helper: ops.helper, tracked_object: ops.tracked_object, callback: ops.callback)
            service_publications_.append(pub)

            let uri = "rosrpc://\(Ros.network.getHost()):\(connection_manager_.getTCPPort())"
            let params = XmlRpcValue(anyArray: [Ros.this_node.getName(),ops.service,uri,xmlrpc_manager_.serverURI])
            do {
                try Master.shared.execute(method: "registerService", request: params).wait()
            } catch {
                ROS_ERROR(error.localizedDescription)
            }

            return true

        }

        func unadvertiseService(serv_name: String) -> Bool {
            if shutting_down.load() {
                return false
            }

            guard let p = service_publications_.index(where: {$0.name == serv_name}) else {
                return false
            }

            service_publications_.remove(at: p, completion: { (sp) in
                self.unregisterService(service: sp.name)
                ROS_DEBUG( "shutting down service [\(sp.name)]")
                sp.drop()
            })

            return true
        }

        private func unregisterService(service: String) {
            let args = XmlRpcValue(anyArray: [Ros.this_node.getName(),service,"rosrpc://\(Ros.network.getHost()):\(connection_manager_.getTCPPort())"])
            do {
                let response = try Master.shared.execute(method: "unregisterService", request: args).wait()
                ROS_DEBUG("response: \(response)")
            } catch {
                ROS_ERROR("error during unregisterService \(error)")
            }
        }

        func isServiceAdvertised(serv_name: String) -> Bool {
            return service_publications_.first(where: { $0.name == serv_name && !$0.dropped_ })  != nil
        }

        func lookupServicePublication(service: String) -> ServiceProtocol? {
            return service_publications_.first(where: {$0.name == service})
        }

        var response = M_string()

        func callback(m: M_string) {
            ROS_DEBUG("callback: \(m)")
            self.response = m

        }

        final class ServiceHandler: ChannelInboundHandler {
            enum ServiceState {
                case header
                case message
            }
            var state : ServiceState = .header

            typealias InboundIn = ByteBuffer

            func channelRead(ctx: ChannelHandlerContext, data: NIOAny) {
                var buffer = self.unwrapInboundIn(data)
                switch state {
                case .header:
                    guard let len : UInt32 = buffer.readInteger(endianness: .little) else {
                        fatalError()
                    }
                    precondition(len <= buffer.readableBytes)

                    var readMap = [String:String]()

                    while buffer.readableBytes > 0 {
                        guard let topicLen : UInt32 = buffer.readInteger(endianness: .little) else {
                            ROS_DEBUG("Received an invalid TCPROS header.  invalid string")
                            fatalError()
                        }

                        guard let line = buffer.readString(length: Int(topicLen)) else {
                            ROS_DEBUG("Received an invalid TCPROS header.  Each line must have an equals sign.")
                            fatalError()
                        }

                        guard let eq = line.index(of: "=") else {
                            ROS_DEBUG("Received an invalid TCPROS header.  Each line must have an equals sign.")
                            fatalError()
                        }
                        let key = String(line.prefix(upTo: eq))
                        let value = String(line.suffix(from: eq).dropFirst())
                        readMap[key] = value
                    }
                    ROS_DEBUG(readMap.debugDescription)
                    state = .message
                case .message:
                    guard let ok : UInt8 = buffer.readInteger(endianness: .little) else {
                        fatalError()
                    }
                    guard let len : UInt32 = buffer.readInteger(endianness: .little) else {
                        fatalError()
                    }
                    precondition(len <= buffer.readableBytes)
                    if let rawMessage = buffer.readBytes(length: Int(len)) {
                        let m = SerializedMessage(buffer: rawMessage)
                    }
                }
            }


        }

        func createServiceServerLink(service: String, persistent: Bool,
                                     request_md5sum: String, response_md5sum: String,
                                     header_values: M_string?) -> ServiceServerLink?
        {

            if shutting_down.load() {
                return nil
            }

            var serv_port : UInt16 = 0
            var serv_host = ""
            guard lookupService(name: service,serv_host: &serv_host,serv_port: &serv_port) else {
                return nil
            }

            let trans = nio.TransportTCP(pipeline: [])

            let c = trans.connect(host: serv_host, port: Int(serv_port)).map { (channel) -> ServiceServerLink in
                let client = ServiceServerLink(service_name: service, persistent: persistent, request_md5sum: request_md5sum, response_md5sum: response_md5sum, header_values: header_values)
                channel.pipeline.add(handler: client)
                client.initialize(channel: channel)
                return client
            }

            do {
                return try c.wait()
            } catch {
                ROS_ERROR("\(error)")
                return nil
            }

        }

        func removeServiceServerLink(client: ServiceServerLink)  {
            if !shutting_down.load() {
                service_server_links_.remove(where: { $0 === client })
            }
        }

        func lookupService(name: String, serv_host: inout String, serv_port: inout UInt16) -> Bool {
            let args = XmlRpcValue(anyArray: [Ros.this_node.getName(),name])
            do {
                let payload = try Master.shared.execute(method: "lookupService", request: args).wait()
                guard payload.valid() else {
                    ROS_DEBUG("lookupService: Invalid server URI returned from master")
                    return false
                }

                let serv_uri = payload.string
                if serv_uri.isEmpty {
                    ROS_DEBUG("lookupService: Empty server URI returned from master")
                    return false
                }

                if !Ros.network.splitURI(uri: serv_uri, host: &serv_host, port: &serv_port) {
                    ROS_DEBUG("lookupService: Bad service uri [\(serv_uri)]")

                    return false
                }

                return true

            }

            catch {
                ROS_ERROR("lookupService: \(error)")
            }

            
            return false
        }
    }
    
