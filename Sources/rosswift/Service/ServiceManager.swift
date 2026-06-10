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
import Foundation


internal final class ServiceManager: RosManager {
    let isShuttingDown = ManagedAtomic(false)
    let servicePublications = SynchronizedArray<ServiceProtocol>()
    let serviceServerLinks = SynchronizedArray<ServiceServerLink>()
    
    let rosID: RosID
    
    init(rosID: RosID) {
        self.rosID = rosID
    }
    
    deinit {
        shutdown()
    }
    
    func start() async {
        isShuttingDown.store(false, ordering: .relaxed)
        ROS_DEBUG("servicemanager start")
    }
    
    func shutdown() {
        guard let services = collectUnregistrations() else { return }
        // Sync entry point: fire-and-forget the master round-trips.
        Task { [self] in
            for name in services { await unregisterService(service: name) }
        }
    }

    /// Async variant of `shutdown()` that awaits the in-flight master
    /// `unregisterService` calls before returning.
    func shutdownAsync() async {
        guard let services = collectUnregistrations() else { return }
        await withTaskGroup(of: Void.self) { group in
            for name in services {
                group.addTask { await self.unregisterService(service: name) }
            }
        }
    }

    /// Performs the synchronous teardown bookkeeping exactly once (drop
    /// publications, close server links) and returns the service names that
    /// still need a master `unregisterService` round-trip. Returns nil if
    /// shutdown was already in progress.
    private func collectUnregistrations() -> [String]? {
        guard isShuttingDown.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged else {
            return nil
        }

        ROS_DEBUG("ServiceManager::shutdown(): unregistering our advertised services")

        let localpub = servicePublications.all()
        servicePublications.removeAll()

        var names = [String]()
        for pub in localpub {
            ROS_DEBUG("shutdown service \(pub.name)")
            names.append(pub.name)
            pub.dropService()
        }

        serviceServerLinks.removeAll { links in
            links.forEach {
                let _ = $0.channel?.close()
            }
        }
        return names
    }
    
    func advertiseService<MReq: ServiceRequestMessage, MRes: ServiceResponseMessage>(_ ops: AdvertiseServiceOptions<MReq, MRes>) async -> Bool {
        if isShuttingDown.load(ordering: .relaxed) {
            return false
        }
        
        if isServiceAdvertised(name: ops.service) {
            ROS_ERROR("Tried to advertise a service that is already advertised in this node [\(ops.service)]")
            return false
        }
        
        let pub = ServicePublication(options: ops)
        servicePublications.append(pub)
        
        guard let ros = Ros.getGlobalRos(for: rosID) else {
            fatalError()
        }
        
        let uri = "rosrpc://\(ros.network.gHost):\(ros.connectionManager.port)"
        let params = XmlRpcValue(anyArray: [ros.name, ops.service, uri, ros.xmlrpcManager.serverURI])
        do {
            let _ = try await ros.master.execute(method: "registerService", request: params)
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
            Task { await self.unregisterService(service: servicePub.name) }
            ROS_DEBUG( "shutting down service [\(servicePub.name)]")
            servicePub.dropService()
        })
        
        return true
    }
    
    private func unregisterService(service: String) async {
        guard let ros = Ros.getGlobalRos(for: rosID) else {
            fatalError()
        }

        let args = XmlRpcValue(anyArray:
                                [ros.name,
                                 service,
                                 "rosrpc://\(ros.network.gHost):\(ros.connectionManager.port)"])
        do {
            _ = try await ros.master.execute(method: "unregisterService", request: args)
        } catch {
            if isShuttingDown.load(ordering: .relaxed) {
                ROS_DEBUG("unregisterService \(service) failed during shutdown: \(error)")
            } else {
                ROS_ERROR("Couldn't unregister service [\(service)]: \(error)")
            }
        }
    }
    
    func isServiceAdvertised(name: String) -> Bool {
        return servicePublications.first(where: { $0.name == name && !$0.isDropped }) != nil
    }
    
    func lookupServicePublication(service: String) -> ServiceProtocol? {
        return servicePublications.first(where: { $0.name == service })
    }
    
    func createServiceServerLink(service: String,
                                 persistent: Bool,
                                 requestMd5sum: String,
                                 responseMd5sum: String,
                                 headerValues: StringStringMap?) async -> ServiceServerLink? {
        
        if isShuttingDown.load(ordering: .relaxed) {
            return nil
        }
        
        guard let server = await lookupService(name: service) else {
            return nil
        }
        
        let trans = TransportTCP()
        
        guard let ros = Ros.getGlobalRos(for: rosID) else {
            fatalError()
        }
        
        let c = trans.connect(host: server.host, port: Int(server.port)).map { channel -> ServiceServerLink in
            let client = ServiceServerLink(ros: ros,
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
            return try await c.get()
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
    
    func lookupService(name: String) async -> (host: String, port: UInt16)? {
        guard let ros = Ros.getGlobalRos(for: rosID) else {
            fatalError()
        }
        
        let args = XmlRpcValue(anyArray: [ros.name, name])
        do {
            let payload = try await ros.master.execute(method: "lookupService", request: args)
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
            
        } catch let error {
            ROS_DEBUG(("lookup: \(error)"))
        }
        
        return nil
    }
}
