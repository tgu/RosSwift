//
//  ServiceServer.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-24.
//

import Atomics

public final class ServiceServer {
    public let service: String
    unowned var node: NodeHandle
    var isUnadvertised = ManagedAtomic(false)

    internal init(service: String, node: NodeHandle) {
        self.service = service
        self.node = node
    }

    deinit {
        unadvertise()
    }

    func unadvertise() {
        if isUnadvertised.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged {
            _ = node.ros.serviceManager.unadvertiseService(name: service)
        }
    }

    func isValid() -> Bool {
        return !isUnadvertised.load(ordering: .relaxed)
    }


    func shutdown() {
        unadvertise()
    }

}
