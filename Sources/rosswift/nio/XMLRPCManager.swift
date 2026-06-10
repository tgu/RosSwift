//
//  XMLRPCManager.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-21.
//

import Foundation
import NIO
import rpcobject


typealias XMLRPCFunc = @Sendable (XmlRpcValue) -> XmlRpcValue

struct XmlRpc {
    static func responseInt(code: Int, msg: String, response: Int) -> XmlRpcValue {
        return XmlRpcValue(anyArray: [code, msg, response])
    }
}

final class XMLRPCManager: RosManager {
    let server: XMLRPCServer
    let host: String

    var serverPort: Int32 { return server.serverPort }
    var serverURI: String { return "http://\(host):\(serverPort)/" }

    internal init(host: String) {
        server = XMLRPCServer(group: threadGroup)
        self.host = host
    }

    func start() async {
        await server.bindAndListen(host: host, port: 0)
        bind(function: "getPid") { _ in
            XmlRpc.responseInt(code: 1, msg: "", response: Int(getpid()))
        }
    }

    func shutdown() {
        // Sync API: clear the method registry so any in-flight requests get an
        // empty response, and fire-and-forget a Task to gracefully close the
        // listener via the server's NIO close path.
        server.removeAll()
        Task { await server.shutdown() }
    }

    /// Async variant that awaits the listener close (graceful drain via
    /// ServerQuiescingHelper). Use from `Ros.shutdownAsync()`.
    func shutdownAsync() async {
        server.removeAll()
        await server.shutdown()
    }

    @discardableResult
    func bind(function: String, cb: @escaping XMLRPCFunc) -> Bool {
        return server.add(method: cb, named: function)
    }

    func unbind(function: String) {
        server.remove(methodName: function)
    }
}
