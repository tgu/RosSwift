//
//  XmlRpcServer.swift
//  swiftros
//
//  Created by Thomas Gustafsson on 2018-03-22.
//

import Foundation
import NIO

extension SocketAddress {
    var host: String {
        switch self {
        case .v4(let addr):
            return addr.host
        case .v6(let addr):
            return addr.host
        case .unixDomainSocket:
            return ""
        }
    }
}

final class MessageHandler: ChannelInboundHandler {
    typealias InboundIn = ByteBuffer
    typealias OutboundOut = ByteBuffer
    weak var server_ : XMLRPCServer!

    func generateResponse(resultXML: String) -> String {
        let rs = "<?xml version=\"1.0\"?>\r\n<methodResponse><params><param>"
        let re = "</param></params></methodResponse>\r\n"
        let body = rs + resultXML + re
        let header = generateHeader(body: body)
        let _response = header + body
        ROS_DEBUG("generateResponse: -----\n\(_response)\n-----\n")
        return _response
    }


    func generateHeader(body: String) -> String {
        let header = "HTTP/1.1 200 OK\r\nServer: "
            + XMLRPC_VERSION
            + "\r\n"
            + "Content-Type: text/xml\r\n"
            + "Content-length: "
            + "\(body.lengthOfBytes(using: .utf8))\r\n\r\n"
        return header
    }

    func channelActive(ctx: ChannelHandlerContext) {
        ROS_DEBUG("xmlrpcserver channelActive")
    }

    func channelInactive(ctx: ChannelHandlerContext) {
        ROS_DEBUG("xmlrpcserver channelInactive")

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

//        guard let data = request.data(using: .utf8) else {
//            return
//        }
//        var obj : XMLRPCRequest? = nil
//        do {
//            obj = try XMLRPCSerialization.xmlrpcRequest(from: data)
//        } catch let error {
//            ROS_ERROR(error.localizedDescription)
//            fatalError()
//        }
        let obj = XMLRPCManager.parseRequest(xml: request)
        let methodName = obj.method
        let params = XmlRpcValue(anyArray: obj.params)

        let response = self.executeMethod(methodName: methodName, params: params)
        let xml = response.toXml()
        let responseXML = self.generateResponse(resultXML: xml)
        var outbuffer = ctx.channel.allocator.buffer(capacity: responseXML.utf8.count)
        let written = outbuffer.write(string: responseXML)
        assert(written == responseXML.utf8.count)

        ctx.writeAndFlush(self.wrapOutboundOut(outbuffer)).whenFailure { (error) in
            ROS_ERROR("write failed to \(ctx.remoteAddress)\nerror: \(error))")
        }
    }

    func executeMethod(methodName: String, params: XmlRpcValue) -> XmlRpcValue {
        if let method = server_.findMethod(name: methodName) {
            do {
                return try method.execute(params: params)
            } catch {
                ROS_ERROR(error.localizedDescription)
            }
        }
        return XmlRpcValue(str: "")
    }


}

final class XMLRPCServer {
    let handler : MessageHandler
    var channel : Channel? = nil
    var boot : ServerBootstrap? = nil
    var methods = [String: XmlRpcServerMethod]()

    init(group: EventLoopGroup) {
        handler = MessageHandler()

        boot = ServerBootstrap(group: group)
            // Specify backlog and enable SO_REUSEADDR for the server itself
            .serverChannelOption(ChannelOptions.backlog, value: 256)
            .serverChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)

            // Set the handlers that are appled to the accepted Channels
            .childChannelInitializer { channel in
                channel.pipeline.add(handler: self.handler)
            }

            // Enable TCP_NODELAY and SO_REUSEADDR for the accepted Channels
            .childChannelOption(ChannelOptions.socket(IPPROTO_TCP, TCP_NODELAY), value: 1)
            .childChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
            .childChannelOption(ChannelOptions.maxMessagesPerRead, value: 16)
            .childChannelOption(ChannelOptions.recvAllocator, value: AdaptiveRecvByteBufferAllocator())

        handler.server_ = self


    }

    func bindAndListen(port: Int) {
        do {
            channel = try boot?.bind(host: Ros.network.getHost(), port: 0).wait()
        } catch {
            ROS_ERROR("bind failed to [\(Ros.network.getHost())], \(error)")
        }
    }

    func addMethod(method: XmlRpcServerMethod) {
        if methods[method.name] != nil {
            methods.removeValue(forKey: method.name)
        }
        methods[method.name] = method
    }

    func removeMethod(method: XmlRpcServerMethod) {
        methods.removeValue(forKey: method.name)
    }

    func removeMethod(methodName: String) {
        methods.removeValue(forKey: methodName)
    }

    func findMethod(name: String) -> XmlRpcServerMethod? {
        return methods[name]
    }

    
}



