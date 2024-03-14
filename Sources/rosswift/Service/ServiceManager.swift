//
//  ServiceManager.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-05.
//

import NIO
import Atomics
import StdMsgs
import rpcobject
import RosNetwork

    internal final class ServiceManager {
        let isShuttingDown = ManagedAtomic(false)
        var servicePublications = SynchronizedArray<ServiceProtocol>()
        var serviceServerLinks = SynchronizedArray<ServiceServerLink>()

        unowned var ros: Ros!

        deinit {
            shutdown()
        }

        func start(ros: Ros) {
            self.ros = ros
            isShuttingDown.store(false, ordering: .relaxed)
            ROS_DEBUG("servicemanager start")
        }

        func shutdown() {
            guard isShuttingDown.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged else {
                return
            }

            ROS_DEBUG("ServiceManager::shutdown(): unregistering our advertised services")

            let localpub = servicePublications.all()
            servicePublications.removeAll()

            localpub.forEach {
                ROS_DEBUG("shutdown service \($0.name)")
                self.unregisterService(service: $0.name)
                $0.dropService()
            }

            serviceServerLinks.removeAll { links in
                links.forEach {
                    let _ = $0.channel?.close()
                }
            }
        }

        func advertiseService<MReq: ServiceRequestMessage, MRes: ServiceResponseMessage>(_ ops: AdvertiseServiceOptions<MReq, MRes>) -> Bool {
            if isShuttingDown.load(ordering: .relaxed) {
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

            let uri = "rosrpc://\(ros.network.gHost):\(ros.connectionManager.port)"
            let params = XmlRpcValue(anyArray: [ros.name, ops.service, uri, ros.xmlrpcManager.serverURI])
            do {
                let _ = try ros.master.execute(method: "registerService", request: params).wait()
            } catch {
                ROS_ERROR(error.localizedDescription)
            }

            return true

        }

        func unadvertiseService(name: String) -> Bool {
            if isShuttingDown.load(ordering: .relaxed) {
                return false
            }

            guard let pub = servicePublications.index(where: { $0.name == name }) else {
                return false
            }

            servicePublications.remove(at: pub, completion: { servicePub in
                self.unregisterService(service: servicePub.name)
                ROS_DEBUG( "shutting down service [\(servicePub.name)]")
                servicePub.dropService()
            })

            return true
        }

        private func unregisterService(service: String) {
            let args = XmlRpcValue(anyArray:
                [ros.name,
                service,
                "rosrpc://\(ros.network.gHost):\(ros.connectionManager.port)"])
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

            if isShuttingDown.load(ordering: .relaxed) {
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
            if !isShuttingDown.load(ordering: .relaxed) {
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

                guard let server = RosNetwork.splitURI(uri: servURI) else {
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
