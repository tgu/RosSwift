//
//  XmlRpcServer.swift
//  swiftros
//
//  Created by Thomas Gustafsson on 2018-03-22.
//

import Foundation
import NIO
import NIOHTTP1

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

private func httpResponseHead(request: HTTPRequestHead, status: HTTPResponseStatus, headers: HTTPHeaders = HTTPHeaders()) -> HTTPResponseHead {
    var head = HTTPResponseHead(version: request.version, status: status, headers: headers)
    let connectionHeaders: [String] = head.headers[canonicalForm: "connection"].map { $0.lowercased() }

    if !connectionHeaders.contains("keep-alive") && !connectionHeaders.contains("close") {
        // the user hasn't pre-set either 'keep-alive' or 'close', so we might need to add headers

        switch (request.isKeepAlive, request.version.major, request.version.minor) {
        case (true, 1, 0):
            // HTTP/1.0 and the request has 'Connection: keep-alive', we should mirror that
            head.headers.add(name: "Connection", value: "keep-alive")
        case (false, 1, let n) where n >= 1:
            // HTTP/1.1 (or treated as such) and the request has 'Connection: close', we should mirror that
            head.headers.add(name: "Connection", value: "close")
        default:
            // we should match the default or are dealing with some HTTP that we don't support, let's leave as is
            ()
        }
    }
    return head
}


final class HTTPHandler: ChannelInboundHandler {
    public typealias InboundIn = HTTPServerRequestPart
    public typealias OutboundOut = HTTPServerResponsePart
    weak var server: XMLRPCServer!

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

    private var buffer: ByteBuffer! = nil
    private var keepAlive = false
    private var state = State.idle

    private var infoSavedRequestHead: HTTPRequestHead?

    public init(server: XMLRPCServer) {
        self.server = server
    }


    private func completeResponse(_ context: ChannelHandlerContext, trailers: HTTPHeaders?, promise: EventLoopPromise<Void>?) {
        self.state.responseComplete()

        let promise = self.keepAlive ? promise : (promise ?? context.eventLoop.makePromise())
        if !self.keepAlive {
            promise!.futureResult.whenComplete { (_: Result<Void, Error>) in context.close(promise: nil) }
        }

        context.writeAndFlush(self.wrapOutboundOut(.end(trailers)), promise: promise)
    }

    func channelRead(context: ChannelHandlerContext, data: NIOAny) {
        let reqPart = self.unwrapInboundIn(data)

        switch reqPart {
        case .head(let request):
            self.keepAlive = request.isKeepAlive
            self.state.requestReceived()
            self.infoSavedRequestHead = request
            self.buffer.clear()

        case .body(buffer: var buf):
            self.buffer.writeBuffer(&buf)
            break
        case .end:
            self.state.requestComplete()
            guard let str = self.buffer.readString(length: self.buffer.readableBytes) else {
                fatalError()
            }

            let obj = XMLRPCManager.parseRequest(xml: str)
            let methodName = obj.method
            let params = XmlRpcValue(anyArray: obj.params)

            let resp = self.executeMethod(methodName: methodName, params: params)
            let xml = resp.toXml()
            let responseXML = self.generateResponse(resultXML: xml)

            self.buffer.clear()
            self.buffer.writeString(responseXML)

            var responseHead = httpResponseHead(request: self.infoSavedRequestHead!, status: HTTPResponseStatus.ok)
            responseHead.headers.add(name: "content-length", value: "\(self.buffer!.readableBytes)")
            responseHead.headers.add(name: "content-type", value: "text/xml")
            let response = HTTPServerResponsePart.head(responseHead)
            context.write(self.wrapOutboundOut(response), promise: nil)
            context.write(self.wrapOutboundOut(.body(.byteBuffer(self.buffer))), promise: nil)
            self.completeResponse(context, trailers: nil, promise: nil)
        }
    }

    private func parseRequest(xml: String) -> (method: String, params: [XmlRpcValue]) {
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


    func executeMethod(methodName: String, params: XmlRpcValue) -> XmlRpcValue {
        if let method = server.findMethod(name: methodName) {
            do {
                return try method.execute(params: params)
            } catch {
                ROS_ERROR(error.localizedDescription)
            }
        }
        return XmlRpcValue(str: "")
    }

    func generateResponse(resultXML: String) -> String {
        let res = "<?xml version=\"1.0\"?>\r\n<methodResponse><params><param>"
        let req = "</param></params></methodResponse>\r\n"
        let body = res + resultXML + req
        return body
        //        let header = generateHeader(body: body)
        //        let response = header + body
        //        return response
    }

    func generateHeader(body: String) -> String {
        let header = "HTTP/1.1 200 OK\r\nServer: "
            + xmlrpcVersion
            + "\r\n"
            + "Content-Type: text/xml\r\n"
            + "Content-length: "
            + "\(body.lengthOfBytes(using: .utf8))\r\n\r\n"
        return header
    }

    func channelReadComplete(context: ChannelHandlerContext) {
        context.flush()
    }

    func handlerAdded(context: ChannelHandlerContext) {
        self.buffer = context.channel.allocator.buffer(capacity: 0)
    }

    func userInboundEventTriggered(context: ChannelHandlerContext, event: Any) {
        switch event {
        case let evt as ChannelEvent where evt == ChannelEvent.inputClosed:
            // The remote peer half-closed the channel. At this time, any
            // outstanding response will now get the channel closed, and
            // if we are idle or waiting for a request body to finish we
            // will close the channel immediately.
            switch self.state {
            case .idle, .waitingForRequestBody:
                context.close(promise: nil)
            case .sendingResponse:
                self.keepAlive = false
            }
        default:
            context.fireUserInboundEventTriggered(event)
        }
    }
}

final class XMLRPCServer {
    var channel: Channel?
    var boot: ServerBootstrap?
    var methods = [String: XmlRpcServerMethod]()

    init(group: EventLoopGroup) {

        self.boot = ServerBootstrap(group: group)
            // Specify backlog and enable SO_REUSEADDR for the server itself
            .serverChannelOption(ChannelOptions.backlog, value: 256)
            .serverChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)

            // Set the handlers that are applied to the accepted Channels
            .childChannelInitializer { channel in
                channel.pipeline.configureHTTPServerPipeline(withErrorHandling: true).flatMap {
                    channel.pipeline.addHandler(HTTPHandler(server: self))
                }
            }

            // Enable TCP_NODELAY and SO_REUSEADDR for the accepted Channels
            .childChannelOption(ChannelOptions.socket(IPPROTO_TCP, TCP_NODELAY), value: 1)
            .childChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
            .childChannelOption(ChannelOptions.maxMessagesPerRead, value: 1)
            .childChannelOption(ChannelOptions.allowRemoteHalfClosure, value: true)

    }

    func bindAndListen(host: String, port: Int) {
        do {
            channel = try boot?.bind(host: host, port: port).wait()
            let p = channel?.localAddress?.port ?? 0
            ROS_DEBUG("Service up and running on \(host):\(p)")
        } catch {
            ROS_ERROR("bind failed to [\(host)], \(error)")
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
