//
//  ServicePublication.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-10.
//

import Foundation
import StdMsgs
import Atomics

internal enum ServiceProcessResult {
    case success(Message)
    case failure(String)
}

internal protocol ServiceProtocol: AnyObject, Sendable {
    var name: String { get }
    var isDropped: Bool { get }
    var md5sum: String { get }
    var requestDataType: String { get }
    var responseDataType: String { get }
    var dataType: String { get }
    func dropService()
    func addServiceClientLink(link: ServiceClientLink)
    func removeServiceClientLink(_ link: ServiceClientLink)
    func processRequest(buf: [UInt8]) -> ServiceProcessResult

}

// `weak var trackedObject` blocks checked `Sendable`: weak class references
// are mutated implicitly by ARC, so the compiler can't prove thread-safety.
// The runtime guarantees atomic load/store of weak refs, so `@unchecked` is
// honest here. Everything else on this class is `let` + Sendable.
internal final class ServicePublication<MReq: ServiceRequestMessage, MRes: ServiceResponseMessage>: ServiceProtocol, @unchecked Sendable {
    
    typealias CallFcn = @Sendable (MReq) -> MRes?
    let call: CallFcn
    let name: String
    let _isDropped = ManagedAtomic(false)
    weak var trackedObject: TrackableObject?
    let clientLinks = SynchronizedArray<ServiceClientLink>()
    
    var md5sum: String { return MReq.srvMd5sum }
    var dataType: String { return MReq.srvDatatype }
    var requestDataType: String { return MReq.datatype }
    var responseDataType: String { return MRes.datatype }
    var isDropped: Bool { _isDropped.load(ordering: .relaxed)}
    let hasTrackedObject: Bool
    
    init(options: AdvertiseServiceOptions<MReq,MRes>) {
        self.call = options.callback
        self.name = options.service
        self.trackedObject = options.trackedObject
        self.hasTrackedObject = options.trackedObject != nil
    }
    
    init(name: String, trackedObject: TrackableObject?, callback: @escaping CallFcn) {
        self.call = callback
        self.name = name
        self.trackedObject = trackedObject
        self.hasTrackedObject = trackedObject != nil
    }
    
    deinit {
        dropService()
    }
    
    func dropService() {
        if !_isDropped.compareExchange(expected: false, desired: true, ordering: .relaxed).original {
            ROS_DEBUG("drop service \(name)")
            dropAllConnections()
        }
    }
    
    func processRequest(buf: consuming [UInt8]) -> ServiceProcessResult {
        if hasTrackedObject && trackedObject == nil {
            return .failure("service tracked object has been destroyed")
        }

        let m = SerializedMessage(buffer: buf)
        let request: MReq
        do {
            request = try deserializeMessage(m: m)
        } catch {
            let msg = "failed to deserialize service request for \(name): \(error)"
            ROS_ERROR(msg)
            return .failure(msg)
        }

        guard let response: MRes = call(request) else {
            return .failure("service handler for \(name) returned no response")
        }
        return .success(response)
    }
    
    func addServiceClientLink(link: ServiceClientLink) {
        clientLinks.append(link)
    }
    
    func removeServiceClientLink(_ link: ServiceClientLink) {
        clientLinks.remove(where: { $0.id == link.id })
    }
    
    func dropAllConnections() {
        clientLinks.removeAll(completion: { elems in
            elems.forEach { $0.dropServiceClient() }
        })
    }
    
}
