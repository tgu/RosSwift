//
//  nioXMLRPCManager.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-21.
//

import Foundation
import NIO


typealias XMLRPCFunc = (XmlRpcValue) -> XmlRpcValue

struct XmlRpc {
    static func responseInt(code: Int, msg: String, response: Int) -> XmlRpcValue {
        return XmlRpcValue(anyArray: [code, msg, response])
    }
}

final class XMLRPCManager {

    var functions = [String: FunctionInfo]()
    var server = XMLRPCServer(group: threadGroup)
    var isUnbindRequested = false
    let host: String
    let functionsQueue = DispatchQueue(label: "functionsQueue")

    var serverPort: Int32 { return Int32(server.channel?.localAddress?.port ?? 0) }
    var serverURI: String { return "http://\(host):\(serverPort)/" }

    typealias XLMRPCFunction = (XmlRpcValue) -> XmlRpcValue
    struct FunctionInfo {
        let name: String
        let function: XLMRPCFunction
        let wrapper: XMLRPCCallWrapper
    }

    internal init(host: String) {
        self.host = host
    }

    func start(host: String) {
        server.bindAndListen(host: host, port: 0)
    }

    func shutdown() {
        functions.removeAll()
    }

    @discardableResult
    func bind(function: String, cb: @escaping XMLRPCFunc) -> Bool {
        var ok = true
        functionsQueue.sync {
            if functions[function] != nil {
                ROS_ERROR("function already bound")
                ROS_ERROR("\(functions)")
                ok = false
            } else {
                let wrap = XMLRPCCallWrapper(function: function, cb: cb, server: server)
                let info = FunctionInfo(name: function, function: cb, wrapper: wrap )

                functions[function] = info
            }
        }

        return ok
    }

    func unbind(function: String) {
        isUnbindRequested = true
        functionsQueue.sync {
            functions.removeValue(forKey: function)
            isUnbindRequested = false
        }
    }

    func getPid(params: XmlRpcValue) -> XmlRpcValue {
        return XmlRpcValue(anyArray: [2, "", Int(getpid())])
    }

    static func parseRequest(xml: String) -> (method: String, params: [XmlRpcValue]) {
        var xmlSeq = xml.dropFirst(0)
        var params = [XmlRpcValue]()
        let methodName = XmlRpcUtil.parseTag(from: .methodname, to: .endMethodname, xml: &xmlSeq)
        if !methodName.isEmpty && XmlRpcUtil.findTag(tag: .params, xml: &xmlSeq) {
            while XmlRpcUtil.nextTagIs(tag: .param, xml: &xmlSeq) {
                var v = XmlRpcValue()
                let _ = v.fromXML(xml: &xmlSeq)
                params.append(v)
                _ = XmlRpcUtil.nextTagIs(tag: .endParam, xml: &xmlSeq)
            }
            _ = XmlRpcUtil.nextTagIs(tag: .endParams, xml: &xmlSeq)
        }
        return (methodName, params)
    }

}
