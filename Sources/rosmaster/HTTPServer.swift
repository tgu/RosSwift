//
//  HTTPServer.swift
//  rosmaster
//
//  Created by Thomas Gustafsson on 2019-04-06.
//

import Foundation
import NIO
import NIOConcurrencyHelpers
import NIOHTTP1
import Logging
import rpcobject

fileprivate let logger = Logger(label: "http")

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

let xmlrpcVersion = "XMLRPC++ 0.7"


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

private final class HTTPHandler: ChannelInboundHandler {
    public typealias InboundIn = HTTPServerRequestPart
    public typealias OutboundOut = HTTPServerResponsePart
    private var handler: (String, [XmlRpcValue]) -> XmlRpcValue

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

    public init(handler: @escaping (String, [XmlRpcValue]) -> XmlRpcValue) {
        self.handler = handler
    }

    private func completeResponse(_ context: ChannelHandlerContext, trailers: HTTPHeaders?, promise: EventLoopPromise<Void>?) {
        self.state.responseComplete()

        let promise = keepAlive ? promise : (promise ?? context.eventLoop.makePromise())
        if !keepAlive {
            promise!.futureResult.whenComplete { (_: Result<Void, Error>) in context.close(promise: nil) }
        }

        context.writeAndFlush(wrapOutboundOut(.end(trailers)), promise: promise)
    }

    func channelRead(context: ChannelHandlerContext, data: NIOAny) {
        let reqPart = self.unwrapInboundIn(data)

        switch reqPart {
        case .head(let request):
            keepAlive = request.isKeepAlive
            state.requestReceived()
            infoSavedRequestHead = request
            buffer.clear()

        case .body(buffer: var buf):
            buffer.writeBuffer(&buf)
            break

        case .end:
            state.requestComplete()
            guard let str = buffer.readString(length: buffer.readableBytes) else {
                fatalError()
            }
            let obj = parseRequest(xml: str)
            let resp = executeRequest(method: obj.method, params: obj.params)
            let xml = resp.toXml()
            let responseXML = self.generateResponse(resultXML: xml)
            buffer.clear()
            buffer.writeString(responseXML)

            var head = httpResponseHead(request: infoSavedRequestHead!, status: HTTPResponseStatus.ok)
            head.headers.add(name: "content-length", value: "\(buffer.readableBytes)")
            head.headers.add(name: "content-type", value: "text/xml")
            context.write(wrapOutboundOut(.head(head)), promise: nil)
            context.write(wrapOutboundOut(.body(.byteBuffer(buffer))), promise: nil)
            self.completeResponse(context, trailers: nil, promise: nil)
        }
    }

    //Mark: parser
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


    private func executeRequest(method: String, params: [XmlRpcValue]) -> XmlRpcValue {
        if method == "system.multicall" {
            precondition(params.count == 1)
            let par = params[0]
            let calls = par.size()
            var res = [XmlRpcValue]()
            for i in 0..<calls {
                let x = par[i]
                guard let method = x.dictionary,
                    let methodName = method["methodName"]?.string,
                    let parameters = method["params"]?.array else {
                    return XmlRpcValue(anyArray: [-1,"",0])
                }
                let r = handler(methodName, parameters)
                res.append(r)
            }
            return XmlRpcValue(anyArray: [1,"",res])
        } else {
            return handler(method, params)
        }
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

public final class XMLRPCServer {
    private let group: EventLoopGroup
    private var channel: Channel?
    private var closure: (String, [XmlRpcValue]) -> XmlRpcValue

    init(group: EventLoopGroup, handler: @escaping (String, [XmlRpcValue]) -> XmlRpcValue) {
        self.group = group
        closure = handler
        state = .initializing
    }

    deinit {
        assert(.stopped == self.state)
    }

    func start(host: String, port: Int) -> EventLoopFuture<XMLRPCServer> {
        assert(.initializing == self.state)
        let bootstrap = ServerBootstrap(group: group)
            // Specify backlog and enable SO_REUSEADDR for the server itself
            .serverChannelOption(ChannelOptions.backlog, value: 256)
            .serverChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)

            // Set the handlers that are applied to the accepted Channels
            .childChannelInitializer { channel in
                channel.pipeline.configureHTTPServerPipeline(withErrorHandling: true).flatMap {
                    channel.pipeline.addHandler(HTTPHandler(handler: self.closure))
                }
            }

            // Enable TCP_NODELAY and SO_REUSEADDR for the accepted Channels
            .childChannelOption(ChannelOptions.socket(IPPROTO_TCP, TCP_NODELAY), value: 1)
            .childChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
            .childChannelOption(ChannelOptions.maxMessagesPerRead, value: 1)
            .childChannelOption(ChannelOptions.allowRemoteHalfClosure, value: true)

        self.state = .starting("\(host):\(port)")
        return bootstrap.bind(host: host, port: port).flatMap { channel in
            self.channel = channel
            self.state = .started
            return channel.eventLoop.makeSucceededFuture(self)
        }
    }

    func stop() -> EventLoopFuture<Void> {
        if .started != self.state {
            return self.group.next().makeFailedFuture(ServerError.notReady)
        }
        guard let channel = self.channel else {
            return self.group.next().makeFailedFuture(ServerError.notReady)
        }
        self.state = .stopping
        channel.closeFuture.whenComplete { _ in
            self.state = .stopped
        }
        return channel.close()
    }

    private var _state = State.initializing
    private let lock = Lock()
    private var state: State {
        get {
            return self.lock.withLock {
                _state
            }
        }
        set {
            self.lock.withLock {
                _state = newValue
                logger.debug("\(self) \(_state)")
            }
        }
    }

    private enum State: Equatable {
        case initializing
        case starting(String)
        case started
        case stopping
        case stopped
    }


}


internal enum ServerError: Error {
    case notReady
    case cantBind
    case timeout
}
