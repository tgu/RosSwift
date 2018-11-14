//
//  nioXMLRPCManager.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-21.
//

import Foundation
import NIO

typealias XMLRPCFunc = (XmlRpcValue) -> XmlRpcValue

struct xmlrpc {
    static func responseInt(code: Int, msg: String, response: Int) -> XmlRpcValue {
        return XmlRpcValue(anyArray: [code,msg,response])
    }
}


final class XMLRPCManager {

    static let instance = XMLRPCManager()

    var handler: MessageHandler?
    var functions = [String:FunctionInfo]()
    var server_ = XMLRPCServer(group: thread_group)
    var unbind_requested = false

    let functionsQueue = DispatchQueue(label: "functionsQueue")

    var serverPort: Int32 { return Int32(server_.channel?.localAddress?.port ?? 0) }
    var serverURI: String { return "http://\(Ros.network.getHost()):\(serverPort)/" }

    typealias XLMRPCFunction = (XmlRpcValue) -> XmlRpcValue
    struct FunctionInfo {
        let name : String
        let function : XLMRPCFunction
        let wrapper : XMLRPCCallWrapper
    }

    private init() {}

    func start() {
        server_.bindAndListen(port: 0)
    }

    func shutdown() {
        functions.removeAll()
    }

    func bind(function_name: String, cb: @escaping XMLRPCFunc) -> Bool {
        var ok = true
        functionsQueue.sync {
            if functions[function_name] != nil {
                ROS_ERROR("function already bound")
                ROS_ERROR("\(functions)")
                ok = false
            } else {
                let wrap = XMLRPCCallWrapper(function_name: function_name, cb: cb, server: server_)
                let info = FunctionInfo(name: function_name, function: cb, wrapper: wrap )

                functions[function_name] = info
            }
        }


        return ok
    }

    func unbind(function_name: String) {
        unbind_requested = true
        functionsQueue.sync {
            functions.removeValue(forKey: function_name)
            unbind_requested = false
        }
    }


    func getPid(params: XmlRpcValue) -> XmlRpcValue {
        return XmlRpcValue(anyArray: [2,"",Int(getpid())])
    }

    static func parseRequest(xml: String) -> (method: String, params: [XmlRpcValue])  {
        var xmlSeq = xml.dropFirst(0)
        var params = [XmlRpcValue]()
        let methodName = XmlRpcUtil.parseTag(from: .METHODNAME_TAG, to: .METHODNAME_ETAG, xml: &xmlSeq)
        if methodName.count > 0 && XmlRpcUtil.findTag(tag: .PARAMS_TAG, xml: &xmlSeq) {
            while XmlRpcUtil.nextTagIs(tag: .PARAM_TAG, xml: &xmlSeq) {
                let v = XmlRpcValue()
                v.fromXML(xml: &xmlSeq)
                params.append(v)
                _ = XmlRpcUtil.nextTagIs(tag: .PARAM_ETAG, xml: &xmlSeq)
            }
            _ = XmlRpcUtil.nextTagIs(tag: .PARAMS_ETAG, xml: &xmlSeq)
        }
        return (methodName,params)
    }

}
