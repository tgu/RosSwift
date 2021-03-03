//
//  XmlRpcServer.swift
//  swiftros
//
//  Created by Thomas Gustafsson on 2018-03-22.
//

import Foundation
import NIO
import NIOHTTP1
import rpcobject

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
    typealias InboundIn = HTTPServerRequestPart
    typealias OutboundOut = HTTPServerResponsePart
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

    init(server: XMLRPCServer) {
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

            let obj = XmlRpcUtil.parseRequest(xml: str)
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

    func executeMethod(methodName: String, params: XmlRpcValue) -> XmlRpcValue {
        if let method = server.find(method: methodName) {
        	return method(params)
        }
        return XmlRpcValue(str: "")
    }

    func generateResponse(resultXML: String) -> String {
        let res = "<?xml version=\"1.0\"?>\r\n<methodResponse><params><param>"
        let req = "</param></params></methodResponse>\r\n"
        let body = res + resultXML + req
        return body
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
    var methods = [String: XMLRPCFunc]()
    let methodsQueue = DispatchQueue(label: "methodsQueue")

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

    func add(method: @escaping XMLRPCFunc, named: String) -> Bool {
        var ok = true
        methodsQueue.sync {
            if methods[named] != nil {
            	ROS_ERROR("function already bound")
            	ROS_ERROR("\(methods)")
                ok = false
            } else {
                methods[named] = method
            }
        }
        return ok
    }

    func removeMethod(method: XmlRpcServerMethod) {
        remove(methodName: method.name)
    }

    func remove(methodName: String) {
        methodsQueue.sync {
            _ = methods.removeValue(forKey: methodName)
        }
    }

    func find(method name: String) -> XMLRPCFunc? {
        return methodsQueue.sync {
            return methods[name]
        }
    }

}
