//
//  ServicePublication.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-10.
//

import Foundation
import StdMsgs

internal protocol ServiceProtocol: AnyObject {
    var name: String { get }
    var isDropped: Bool { get }
    var md5sum: String { get }
    var requestDataType: String { get }
    var responseDataType: String { get }
    var dataType: String { get }
    func dropService()
    func addServiceClientLink(link: ServiceClientLink)
    func removeServiceClientLink(_ link: ServiceClientLink)
    func processRequest(buf: [UInt8]) -> Message?

}

internal final class ServicePublication<MReq: ServiceRequestMessage, MRes: ServiceResponseMessage>: ServiceProtocol {

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
        dropService()
    }

    func dropService() {
        ROS_DEBUG("drop service \(name)")
        // grab a lock here, to ensure that no subscription callback will
        // be invoked after we return
        clientLinksQueue.sync {
            isDropped = true
        }
        dropAllConnections()
    }

    func processRequest(buf: [UInt8]) -> Message? {

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
    }

    func addServiceClientLink(link: ServiceClientLink) {
        clientLinksQueue.sync {
            clientLinks.append(link)
        }
    }

    func removeServiceClientLink(_ link: ServiceClientLink) {
        clientLinksQueue.sync {
            if let it = clientLinks.firstIndex(where: { $0 === link }) {
                clientLinks.remove(at: it)
            }
        }
    }

    func dropAllConnections() {
    // Swap our clientLinks list with a local one so we can only lock for a short period of time, because a
    // side effect of our calling drop() on connections can be re-locking the clientLinks mutex
    var localLinks = [ServiceClientLink]()

        clientLinksQueue.sync {
            swap(&localLinks, &clientLinks)
        }

    }

}
