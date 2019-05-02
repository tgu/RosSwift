//
//  ServicePublication.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-10.
//

import Foundation
import StdMsgs

internal protocol ServiceProtocol: class {
    var name: String { get }
    var isDropped: Bool { get }
    var md5sum: String { get }
    var requestDataType: String { get }
    var responseDataType: String { get }
    var dataType: String { get }
    func drop()
    func addServiceClientLink(link: ServiceClientLink)
    func removeServiceClientLink(_ link: ServiceClientLink)
    func processRequest(buf: [UInt8]) -> ServiceMessage?

}

internal final class ServicePublication<MReq: ServiceMessage, MRes: ServiceMessage>: ServiceProtocol {

    typealias CallFcn = (MReq) -> MRes?
    var call: CallFcn
    var name: String
    var md5sum: String { return MReq.srvMd5sum }
    var dataType: String { return MReq.srvDatatype }
    var requestDataType: String { return MReq.datatype }
    var responseDataType: String { return MRes.datatype }
    var isDropped = false
    var hasTrackedObject: Bool { return trackedObject != nil }
    var trackedObject: AnyObject?

    var clientLinks = [ServiceClientLink]()
    let clientLinksQueue = DispatchQueue(label: "clientLinksQueue")

    init(name: String, trackedObject: AnyObject?, callback: @escaping CallFcn) {
        self.call = callback
        self.name = name
        self.trackedObject = trackedObject
    }

    deinit {
        drop()
    }

    func drop() {
        ROS_DEBUG("drop")
        // grab a lock here, to ensure that no subscription callback will
        // be invoked after we return
        clientLinksQueue.sync {
            isDropped = true
        }
        dropAllConnections()
    }

    final class ServiceCallback: CallbackInterface {

        func call() -> CallResult {
            ROS_ERROR("call() not implemented")

            return .invalid
        }

        func ready() -> Bool {
            return true
        }

        var buffer: [UInt8]
        var link: ServiceClientLink
        var hasTrackedObject: Bool
        var trackedObject: AnyObject?

        init(buf: [UInt8],
             link: ServiceClientLink,
             hasTrackedObject: Bool,
             trackedObject: AnyObject?) {

            self.buffer = buf
            self.link = link
            self.hasTrackedObject = hasTrackedObject
            self.trackedObject = trackedObject
        }

    }

    func processRequest(buf: [UInt8]) -> ServiceMessage? {

        let m = SerializedMessage(buffer: buf)
        do {
            let request: MReq = try deserializeMessage(m: m)
            if let response: MRes = call(request) {                     // TODO: handle false return
                return response
            }
        } catch {
            ROS_ERROR("deserializeMessage of \(m) failed: \(error)")
        }

        return nil
//
//        let cb = ServiceCallback(helper: helper_, buf: buf, num_bytes: num_bytes, link: link, has_tracked_object: has_tracked_object, tracked_object: tracked_object_)
//
//        callback_queue_?.addCallback(callback: cb, owner_id: hash)
    }

    func addServiceClientLink(link: ServiceClientLink) {
        clientLinksQueue.sync {
            clientLinks.append(link)
        }
    }

    func removeServiceClientLink(_ link: ServiceClientLink) {
        clientLinksQueue.sync {
            #if swift(>=4.2)
            if let it = clientLinks.firstIndex(where: { $0 === link }) {
                clientLinks.remove(at: it)
            }
            #else
            if let index = clientLinks.index(where: { $0 === link }) {
                clientLinks.remove(at: index)
            }
            #endif
        }
    }

    func dropAllConnections() {
    // Swap our clientLinks list with a local one so we can only lock for a short period of time, because a
    // side effect of our calling drop() on connections can be re-locking the clientLinks mutex
    var localLinks = [ServiceClientLink]()

        clientLinksQueue.sync {
            swap(&localLinks, &clientLinks)
        }

//        local_links.forEach { $0.connection?.drop(reason: .Destructing)}
    }

}
