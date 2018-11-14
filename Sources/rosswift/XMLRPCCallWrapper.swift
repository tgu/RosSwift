//
//  XMLRPCCallWrapper.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-08.
//

import Foundation

final class XMLRPCCallWrapper: XmlRpcServerMethod {
    let name : String
    let server : XMLRPCServer
    let function : XMLRPCFunc

    init(function_name: String, cb: @escaping XMLRPCFunc, server: XMLRPCServer) {
        self.name = function_name
        self.server = server
        self.function = cb
        server.addMethod(method: self)
    }

    deinit {
        server.removeMethod(method: self)
    }

    func execute(params: XmlRpcValue) throws -> XmlRpcValue {
        return function(params)
    }

    func help() -> String {
        return ""
    }


}
