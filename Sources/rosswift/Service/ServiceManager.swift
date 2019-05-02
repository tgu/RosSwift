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

    internal final class ServiceManager {
//        static let instance = ServiceManager()

        var isShuttingDown = Atomic<Bool>(value: false)
        var servicePublications = SynchronizedArray<ServiceProtocol>()
        var serviceServerLinks = SynchronizedArray<ServiceServerLink>()

        var connectionManager: ConnectionManager { return ros.connectionManager }
        var xmlrpcManager: XMLRPCManager { return ros.xmlrpcManager }
        unowned var ros: Ros!

        internal init() {
        }

        deinit {
            shutdown()
        }

        func start(ros: Ros) {
            self.ros = ros
            isShuttingDown.store(false)
            ROS_DEBUG("servicemanager start")
        }

        func shutdown() {
            guard isShuttingDown.compareAndExchange(expected: false, desired: true) else {
                return
            }

            ROS_DEBUG("ServiceManager::shutdown(): unregistering our advertised services")

            let localpub = servicePublications.all()
            servicePublications.removeAll()

            localpub.forEach {
                ROS_DEBUG("shutdown service \($0.name)")
                self.unregisterService(service: $0.name)
                $0.drop()
            }

            serviceServerLinks.removeAll { links in
                links.forEach {
                    let _ = $0.channel?.close()
                }
            }
        }

        func advertiseService<MReq: ServiceMessage, MRes: ServiceMessage>(_ ops: AdvertiseServiceOptions<MReq, MRes>) -> Bool {
            if isShuttingDown.load() {
                return false
            }

            if isServiceAdvertised(name: ops.service) {
                ROS_ERROR("Tried to advertise a service that is already advertised in this node [\(ops.service)]")
                return false
            }

            let pub = ServicePublication(name: ops.service,
                                         trackedObject: ops.trackedObject,
                                         callback: ops.callback)
            servicePublications.append(pub)

            let uri = "rosrpc://\(ros.network.getHost()):\(connectionManager.getTCPPort())"
            let params = XmlRpcValue(anyArray: [ros.name, ops.service, uri, xmlrpcManager.serverURI])
            do {
                let _ = try ros.master.execute(method: "registerService", request: params).wait()
            } catch {
                ROS_ERROR(error.localizedDescription)
            }

            return true

        }

        func unadvertiseService(name: String) -> Bool {
            if isShuttingDown.load() {
                return false
            }

            guard let pub = servicePublications.index(where: { $0.name == name }) else {
                return false
            }

            servicePublications.remove(at: pub, completion: { servicePub in
                self.unregisterService(service: servicePub.name)
                ROS_DEBUG( "shutting down service [\(servicePub.name)]")
                servicePub.drop()
            })

            return true
        }

        private func unregisterService(service: String) {
            let args = XmlRpcValue(anyArray:
                [ros.name,
                service,
                "rosrpc://\(ros.network.getHost()):\(connectionManager.getTCPPort())"])
            do {
                let response = try ros.master.execute(method: "unregisterService", request: args).wait()
                ROS_DEBUG("response: \(response)")
            } catch {
                ROS_ERROR("error during unregisterService \(error)")
            }
        }

        func isServiceAdvertised(name: String) -> Bool {
            return servicePublications.first(where: { $0.name == name && !$0.isDropped }) != nil
        }

        func lookupServicePublication(service: String) -> ServiceProtocol? {
            return servicePublications.first(where: { $0.name == service })
        }

        var response = StringStringMap()

        func callback(m: StringStringMap) {
            ROS_DEBUG("callback: \(m)")
            self.response = m

        }

        func createServiceServerLink(service: String,
                                     persistent: Bool,
                                     requestMd5sum: String,
                                     responseMd5sum: String,
                                     headerValues: StringStringMap?) -> ServiceServerLink? {

            if isShuttingDown.load() {
                return nil
            }

            guard let server = lookupService(name: service) else {
                return nil
            }

            let trans = TransportTCP()

            let c = trans.connect(host: server.host, port: Int(server.port)).map { channel -> ServiceServerLink in
                let client = ServiceServerLink(ros: self.ros,
                                               serviceName: service,
                                               persistent: persistent,
                                               requestMd5sum: requestMd5sum,
                                               responseMd5sum: responseMd5sum,
                                               headerValues: headerValues)
                let _ = channel.pipeline.addHandler(client)
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

        func removeServiceServerLink(client: ServiceServerLink) {
            if !isShuttingDown.load() {
                serviceServerLinks.remove(where: { $0 === client })
            }
        }

        func lookupService(name: String) -> (host: String, port: UInt16)? {
            let args = XmlRpcValue(anyArray: [ros.name, name])
            do {
                let payload = try ros.master.execute(method: "lookupService", request: args).wait()
                guard payload.valid() else {
                    ROS_DEBUG("lookupService: Invalid server URI returned from master")
                    return nil
                }

                let servURI = payload.string
                if servURI.isEmpty {
                    ROS_DEBUG("lookupService: Empty server URI returned from master")
                    return nil
                }

                guard let server = Network.splitURI(uri: servURI) else {
                    ROS_DEBUG("lookupService: Bad service uri [\(servURI)]")
                    return nil
                }

                return server

            } catch {
                ROS_ERROR("lookupService: \(error)")
            }

            return nil
        }
    }
