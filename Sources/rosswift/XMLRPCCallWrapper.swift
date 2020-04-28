//
//  XMLRPCCallWrapper.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-08.
//

import Foundation
import rpcobject

struct XMLRPCCallWrapper: XmlRpcServerMethod {
    let name: String
    let function: XMLRPCFunc

    init(function: String, cb: @escaping XMLRPCFunc, server: XMLRPCServer) {
        self.name = function
        self.function = cb
        server.addMethod(method: self)
    }

    func execute(params: XmlRpcValue) throws -> XmlRpcValue {
        return function(params)
    }

    func help() -> String {
        return ""
    }

}
