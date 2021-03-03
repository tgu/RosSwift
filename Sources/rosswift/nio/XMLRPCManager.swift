//
//  XMLRPCManager.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-21.
//

import Foundation
import NIO
import rpcobject


typealias XMLRPCFunc = (XmlRpcValue) -> XmlRpcValue

struct XmlRpc {
    static func responseInt(code: Int, msg: String, response: Int) -> XmlRpcValue {
        return XmlRpcValue(anyArray: [code, msg, response])
    }
}

final class XMLRPCManager {
    let server: XMLRPCServer
    let host: String

    var serverPort: Int32 { return Int32(server.channel?.localAddress?.port ?? 0) }
    var serverURI: String { return "http://\(host):\(serverPort)/" }

    internal init(host: String) {
        server = XMLRPCServer(group: threadGroup)
        self.host = host
    }

    func start(host: String) {
        server.bindAndListen(host: host, port: 0)
    }

    func shutdown() {
    }

    @discardableResult
    func bind(function: String, cb: @escaping XMLRPCFunc) -> Bool {
        return server.add(method: cb, named: function)
    }

    func unbind(function: String) {
        server.remove(methodName: function)
    }
}
