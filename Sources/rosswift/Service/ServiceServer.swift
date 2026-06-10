//
//  ServiceServer.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-24.
//

import Atomics

public final class ServiceServer {
    public let service: String
    unowned var manager: ServiceManager
    let isUnadvertised = ManagedAtomic(false)

    internal init(service: String, manager: ServiceManager) {
        self.service = service
        self.manager = manager
    }

    deinit {
        unadvertise()
    }

    func unadvertise() {
        if isUnadvertised.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged {
            _ = manager.unadvertiseService(name: service)
        }
    }

    func isValid() -> Bool {
        return !isUnadvertised.load(ordering: .relaxed)
    }


    func shutdown() {
        unadvertise()
    }

}
