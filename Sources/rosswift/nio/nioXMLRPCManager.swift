//
//  nioXMLRPCManager.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-21.
//

import Foundation
import NIO
import XMLRPCSerialization

typealias XMLRPCFunc = (XmlRpcValue) -> XmlRpcValue

struct xmlrpc {
    static func responseInt(code: Int, msg: String, response: Int) -> XmlRpcValue {
        return XmlRpcValue(anyArray: [code,msg,response])
    }
}


class XMLRPCManager {

    static let instance = XMLRPCManager()

//    var channel : Channel?
    var boot: ServerBootstrap?
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
        ROS_DEBUG("XMLRPCManager start")
        let handler = MessageHandler()
        self.handler = handler

        boot = ServerBootstrap(group: thread_group)
            // Specify backlog and enable SO_REUSEADDR for the server itself
            .serverChannelOption(ChannelOptions.backlog, value: 256)
            .serverChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)

            // Set the handlers that are appled to the accepted Channels
            .childChannelInitializer { channel in
                channel.pipeline.add(handler: handler)
            }

            // Enable TCP_NODELAY and SO_REUSEADDR for the accepted Channels
            .childChannelOption(ChannelOptions.socket(IPPROTO_TCP, TCP_NODELAY), value: 1)
            .childChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
            .childChannelOption(ChannelOptions.maxMessagesPerRead, value: 16)
            .childChannelOption(ChannelOptions.recvAllocator, value: AdaptiveRecvByteBufferAllocator())


        do {
//            channel = try boot?.bind(host: Ros.network.getHost(), port: 0).wait()
            server_.bindAndListen(port: 0)
        } catch {
            ROS_ERROR("failed to bind to [::1:0], \(error)")
        }
        
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


    func getPid(params: XmlRpcValue) -> XmlRpcValue
    {
        return XmlRpcValue(anyArray: [2,"",Int(getpid())])
    }

    func validateXmlrpcResponse(method: String, response: XmlRpcValue,
                                payload: inout XmlRpcValue) -> Bool {

        guard response.isArray else {
            ROS_DEBUG("XML-RPC call [\(method)] didn't return an array")
            return false
        }

        var resp = response
        if resp.size() == 1 {
//            ROS_DEBUG("XML-RPC call [\(method)] did return a 1-element array\nLook at child")

            resp = resp[0]
        }

        if resp.size() != 2 && resp.size() != 3 {
            ROS_DEBUG("XML-RPC call [\(method)] didn't return a 2 or 3-element array")
            return false
        }
        guard case .int(let status_code) = resp[0].value else {
            ROS_DEBUG("XML-RPC call [\(method)] didn't return a int as the 1st element")
            return false
        }
        guard case .string(let status_string) = resp[1].value else {
            ROS_DEBUG("XML-RPC call [\(method)] didn't return a string as the 2nd element")
            return false
        }
        if status_code != 1 {
            ROS_DEBUG("XML-RPC call [\(method)] returned an error (\(status_code): [\(status_string)]")
            return false
        }
        if resp.size() > 2 {
            //            if case .string(let uri) = resp[2].value {
            //                payload.value = .array([XmlRpcValue(str: uri)])
            //            } else {
            payload = resp[2]
            //            }
        } else {
            payload = XmlRpcValue(anyArray: [])
        }
        return true
    }


}



extension XMLRPCManager {

    class MessageHandler: ChannelInboundHandler {
        typealias InboundIn = ByteBuffer
        typealias OutboundOut = ByteBuffer

        private enum State {
            case idle
            case waitingForRequestBody
            case sendingResponse

            mutating func requestReceived() {
                precondition(self == .idle, "Invalid state for request received: \(self)")
                self = .waitingForRequestBody
            }

            mutating func requestComplete() {
                precondition(self == .waitingForRequestBody, "Invalid state for request complete: \(self)")
                self = .sendingResponse
            }

            mutating func responseComplete() {
                precondition(self == .sendingResponse, "Invalid state for response complete: \(self)")
                self = .idle
            }
        }

        private var channels: [ObjectIdentifier: Channel] = [:]
        let connectionManager : Ros.ConnectionManager? = nil
        private var state = State.idle


        init() {}

        func requestTopic(topic: String, protos: XmlRpcValue) -> XmlRpcValue {
            ROS_DEBUG("requestTopic \(topic)")
            for proto_idx in 0..<protos.size() {
                let proto = protos[proto_idx]
                guard case .array = proto.value else {
                    ROS_DEBUG("requestTopic protocol list was not a list of lists")
                    return XmlRpcValue()
                }

                guard case .string(let proto_name) = proto[0].value else {
                    ROS_DEBUG( "requestTopic received a protocol list in which a sublist did not start with a string")
                    return XmlRpcValue()
                }

                if proto_name == "TCPROS" {
                    let host = connectionManager!.channel!.localAddress!.host
                    let port = connectionManager!.channel!.localAddress!.port!
                    let tcpros_params = XmlRpcValue(array: [.init(str: "TCPROS"),
                                                            .init(str: host),
                                                            .init(any: port)])
                    let ret = XmlRpcValue(array: [.init(any: 1),
                                                  .init(str: ""),
                                                  tcpros_params])
                    return ret
                } else {
                    ROS_DEBUG( "an unsupported protocol was offered: [\(proto_name)]")
                }
            }
            return XmlRpcValue()
        }

        func executeMethod(methodName: String, params: XmlRpcValue) -> XmlRpcValue {
            switch methodName {
            case "requestTopic":
                return requestTopic(topic: params[1].string, protos: params[2])
            default:
                return XmlRpcValue(anyArray: [0,"error",0])
            }
        }

        func generateResponse(resultXML: String) -> String {
            let rs = "<?xml version=\"1.0\"?>\r\n<methodResponse><params><param>\r\n"
            let re = "\r\n</param></params></methodResponse>\r\n"
            let body = rs + resultXML + re
            let header = generateHeader(body: body)
            let _response = header + body
            ROS_DEBUG("generateResponse: -----\n\(_response)\n-----\n")
            return _response
        }


        func generateHeader(body: String) -> String {
            let header =     "HTTP/1.1 200 OK\r\nServer: "
                + XMLRPC_VERSION
                + "\r\n"
                + "Content-Type: text/xml\r\n"
                + "Content-length: "
                + "\(body.lengthOfBytes(using: .utf8))\r\n\r\n"
            return header
        }

        func channelActive(ctx: ChannelHandlerContext) {
            ROS_DEBUG("channelActive")
        }

        func channelInactive(ctx: ChannelHandlerContext) {
            ROS_DEBUG("channelInactive")

        }

        public func channelRead(ctx: ChannelHandlerContext, data: NIOAny) {
            var reqPart = self.unwrapInboundIn(data)
            guard let str = reqPart.readString(length: reqPart.readableBytes) else {
                return
            }

            guard let range = str.lowercased().range(of: "content-length: ")  else {
                fatalError("Header not read")
            }

            let content = String(str[range.upperBound..<str.endIndex])
            guard let index = content.index(of: "\r\n") else {
                fatalError()
            }
            guard let length = Int(content[content.startIndex..<index]) else {
                fatalError("length error")
            }

            let contentLength = length

            guard let ind2 = content.index(of: "<") else {
                fatalError()
            }

            let request = String(content[ind2..<content.endIndex])
            if str.contains("HTTP/1.0") {
                fatalError()
            }

            guard let data = request.data(using: .utf8) else {
                return
            }
            var obj : XMLRPCRequest? = nil
            do {
                obj = try XMLRPCSerialization.xmlrpcRequest(from: data)
            } catch let error {
                ROS_ERROR(error.localizedDescription)
                fatalError()
            }
            let methodName = obj!.methodName
            let params = XmlRpcValue(anyArray: obj!.params)

            ROS_DEBUG("executing method [\(methodName)] with parameters: \(params)")
            let response = executeMethod(methodName: methodName, params: params)
            let xml = response.toXml()
            let responseXML = generateResponse(resultXML: xml)
            var outbuffer = ctx.channel.allocator.buffer(capacity: responseXML.utf8.count)
            outbuffer.write(string: responseXML)
            ctx.writeAndFlush(self.wrapOutboundOut(outbuffer)).whenFailure { (error) in
                ROS_DEBUG("write failed to \(ctx.remoteAddress!)\nerror: \(error))")
            }
        }

    }


}
