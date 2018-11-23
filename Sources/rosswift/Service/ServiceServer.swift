//
//  ServiceServer.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-24.
//

import Foundation

public final class ServiceServer {

    final class Impl {
        let service: String
        unowned var node: Ros.NodeHandle
        var isUnadvertised = false

        init(service: String, node: Ros.NodeHandle) {
            self.service = service
            self.node = node
        }

        deinit {
            unadvertise()
        }

        func unadvertise() {
            if !isUnadvertised {
                isUnadvertised = true
                _ = ServiceManager.instance.unadvertiseService(name: service)
            }
        }

        func isValid() -> Bool {
            return !isUnadvertised
        }
    }

    var implementation: Impl?

    init(service: String, node: Ros.NodeHandle) {
        implementation = Impl(service: service, node: node)
    }

    func shutdown() {
        implementation?.unadvertise()
    }

    func getService() -> String {
        return implementation?.service ?? ""
    }

    func isValid() -> Bool {
        return implementation?.isValid() ?? false
    }

}
