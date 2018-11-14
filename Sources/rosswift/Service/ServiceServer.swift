//
//  ServiceServer.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-24.
//

import Foundation

public final class ServiceServer {

    final class Impl {
        let service_ : String
        unowned var node_handle_ : Ros.NodeHandle
        var unadvertised_ = false

        init(service: String, node_handle: Ros.NodeHandle) {
            self.service_ = service
            self.node_handle_ = node_handle
        }

        deinit {
            unadvertise()
        }

        func unadvertise() {
            if !unadvertised_ {
                unadvertised_ = true
                ServiceManager.instance.unadvertiseService(serv_name: service_)
            }
        }

        func isValid() -> Bool {
            return !unadvertised_
        }
    }

    var impl_ : Impl?

    init(service: String, node_handle: Ros.NodeHandle) {
        impl_ = Impl(service: service, node_handle: node_handle)
    }

    func shutdown() {
        impl_?.unadvertise()
    }

    func getService() -> String {
        return impl_?.service_ ?? ""
    }

    func isValid() -> Bool {
        return impl_?.isValid() ?? false
    }



}
