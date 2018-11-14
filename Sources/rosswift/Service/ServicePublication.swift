//
//  ServicePublication.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-10.
//

import Foundation
import StdMsgs

protocol ServiceProtocol: class {
    var name : String { get }
    var dropped_ : Bool { get }
    var md5sum : String { get }
    func isDropped() -> Bool
    var request_data_type : String { get }
    var response_data_type : String { get }
    var data_type : String { get }
    func drop()
    func addServiceClientLink(link: ServiceClientLink)
    func removeServiceClientLink(_ link: ServiceClientLink)
    func processRequest(buf: [UInt8]) -> ServiceMessage?

}

final class ServicePublication<MReq: ServiceMessage, MRes: ServiceMessage>: ServiceProtocol {

//    typealias CallFcn = (MReq, inout MRes) -> Bool
    typealias CallFcn = (MReq) -> MRes?
//    lazy var hash: UInt64 = {
//        UInt64(arc4random())
//    }()
    var call: CallFcn
    var name : String
    var md5sum : String { return MReq.srv_md5sum }
    var data_type : String { return MReq.srv_datatype }
    var request_data_type : String { return MReq.datatype }
    var response_data_type : String { return MRes.datatype }
    var helper_ : ServiceCallbackHelper?
    var dropped_ = false
    var has_tracked_object : Bool { return tracked_object_ != nil }
    var tracked_object_ : VoidFunc?


    var client_links_ = [ServiceClientLink]()
    let client_links_mutex_ = DispatchQueue(label: "client_links_mutex_")


    init(name: String, helper:  ServiceCallbackHelper?, tracked_object: VoidFunc?, callback: @escaping CallFcn) {
        self.call = callback
        self.name = name
        self.helper_ = helper
        self.tracked_object_ = tracked_object
    }

    deinit {
        drop()
    }

    func isDropped() -> Bool {
        return dropped_
    }

    func drop() {
        ROS_DEBUG("drop")
        // grab a lock here, to ensure that no subscription callback will
        // be invoked after we return
        client_links_mutex_.sync {
            dropped_ = true
        }
        dropAllConnections()
    }

    final class ServiceCallback: CallbackInterface {
//        lazy var hash: UInt64 = {
//            UInt64(arc4random())
//        }()

        func call() -> CallResult {
            ROS_ERROR("call() not implemented")

            return .invalid
        }

        func ready() -> Bool {
            return true
        }

        var helper_ : ServiceCallbackHelper?
        var buffer_ : [UInt8]
        var num_bytes_ : UInt32
        var link_ : ServiceClientLink
        var has_tracked_object_ : Bool
        var tracked_object_ : VoidFunc?

        init(helper: ServiceCallbackHelper?, buf: [UInt8], num_bytes: UInt32, link: ServiceClientLink, has_tracked_object: Bool, tracked_object: VoidFunc?) {
            helper_ = helper
            buffer_ = buf
            num_bytes_ = num_bytes
            link_ = link
            has_tracked_object_ = has_tracked_object
            tracked_object_ = tracked_object
        }

    }


    func processRequest(buf: [UInt8]) -> ServiceMessage? {

//        var request = MReq()
//        var response = MRes()
        let m = SerializedMessage(buffer: buf)
        do {
            let request : MReq = try deserializeMessage(m: m)
            if let response : MRes = call(request) {                     // TODO: handle false return
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

    func addServiceClientLink(link: ServiceClientLink)  {
        client_links_mutex_.sync {
            client_links_.append(link);
        }
    }

    func removeServiceClientLink(_ link: ServiceClientLink) {
        client_links_mutex_.sync {
            #if swift(>=4.2)
            if let it = client_links_.firstIndex(where: { $0 === link }) {
                client_links_.remove(at: it)
            }
            #else
            if let index = client_links_.index(where: {$0 === link}) {
                client_links_.remove(at: index)
            }
            #endif
        }
    }

    func dropAllConnections() {
    // Swap our client_links_ list with a local one so we can only lock for a short period of time, because a
    // side effect of our calling drop() on connections can be re-locking the client_links_ mutex
    var local_links = [ServiceClientLink]()

        client_links_mutex_.sync {
            swap(&local_links,&client_links_)
        }

//        local_links.forEach { $0.connection?.drop(reason: .Destructing)}
    }


}
