//
//  Master.swift
//  swiftros
//
//  Created by Thomas Gustafsson on 2018-03-19.
//

import Foundation
import NIO
import NIOConcurrencyHelpers
import Logging
import rpcobject

fileprivate let logger = Logger(label: "master")

let xmlrpcVersion = "XMLRPC++ 0.7"

public enum nio {

struct TopicInfo {
    let name: String
    let dataType: String
}

    internal struct RPCRequest {
        let methodName: String
        let params: XmlRpcValue
    }

    internal struct RPCRequestWrapper {
        let id = UUID()
        let request: RPCRequest
        let promise: EventLoopPromise<RPCResponse>
    }

    internal struct RPCResponse {
        let result: XmlRpcValue

        static let methodresponseTag = "<methodResponse>"

        init?(xml: String) {
            guard let tagIndex = xml.range(of: RPCResponse.methodresponseTag) else {
                logger.error("Invalid response - no methodResponse. Response:\n\(xml)")
                return nil
            }

            var result = XmlRpcValue()

            var data = xml.suffix(from: tagIndex.upperBound)
            if XmlRpcUtil.nextTagIs(tag: .params, xml: &data) && XmlRpcUtil.nextTagIs(tag: .param, xml: &data) {
                if result.fromXML(xml: &data) {
                    self.result = result
                    return
                } else {
                    logger.error("Invalid response value. Response: \(data)")
                }
            } else if XmlRpcUtil.nextTagIs(tag: .fault, xml: &data) {
                logger.error("Invalid response value. Response: \(data)")
            } else {
                logger.error("Invalid response - no param or fault tag. Response \(xml)")
            }

            return nil
        }

    }

    internal enum CodecError: Error {
        case headerNotRead
        case malformedHeader
        case lengthError
        case parsingError
    }

    internal enum ClientError: Error {
        case notReady
        case cantBind
        case timeout
        case connectionResetByPeer
    }

    internal final class RPCCodec: ChannelInboundHandler, ChannelOutboundHandler {
        typealias InboundIn = ByteBuffer
        typealias InboundOut = RPCResponse
        typealias OutboundIn = RPCRequest
        typealias OutboundOut = ByteBuffer

        func channelRead(context: ChannelHandlerContext, data: NIOAny) {
            var buffer = self.unwrapInboundIn(data)
            if let string = buffer.readString(length: buffer.readableBytes) {
                if let r = RPCResponse(xml: string) {
                    return context.fireChannelRead(wrapInboundOut(r))
                }
            }
            context.fireErrorCaught(CodecError.parsingError)
        }

        func write(context: ChannelHandlerContext, data: NIOAny, promise: EventLoopPromise<Void>?) {
            let adr = context.channel.remoteAddress!
            let request = self.unwrapOutboundIn(data)
            let xml = XMLRPCClient.generateRequest(methodName: request.methodName, params: request.params, host: adr.host , port: UInt16(adr.port!))
            var buffer = context.channel.allocator.buffer(capacity: xml.utf8.count)
            buffer.writeString(xml)
            context.write(wrapOutboundOut(buffer), promise: promise)
        }
    }


final class XmlRpcMessageDelimiterCodec: ByteToMessageDecoder {
    public typealias InboundIn = ByteBuffer
    public typealias InboundOut = ByteBuffer

    public var cumulationBuffer: ByteBuffer?

    public func decode(context: ChannelHandlerContext, buffer: inout ByteBuffer) throws -> DecodingState {

        guard let header = buffer.getString(at: buffer.readerIndex, length: buffer.readableBytes) else {
            return .needMoreData
        }

        guard let range = header.lowercased().range(of: "content-length: ")  else {
            return .needMoreData
        }

        let content = String(header[range.upperBound..<header.endIndex])
        guard let index = content.firstIndex(of: "\r\n") else {
            return .needMoreData
        }

        guard let length = Int(content[content.startIndex..<index]) else {
            return .needMoreData
        }

        guard let _ = content.firstIndex(of: "<") else {
            logger.debug("Malformed header")
            return .needMoreData
        }

        if content.count >= length {
            context.fireChannelRead(self.wrapInboundOut(buffer.readSlice(length: buffer.readableBytes)!))
            return .continue
        }

        return .needMoreData
    }

    func decodeLast(context: ChannelHandlerContext, buffer: inout ByteBuffer, seenEOF: Bool) throws -> DecodingState {
        /// set cumulationBuffer to avoid to be called again with the same data
        /// https://github.com/apple/swift-nio/issues/108
        ///
        self.cumulationBuffer = nil
        return .continue
    }
}

final class XmlRpcHandler: ChannelInboundHandler, ChannelOutboundHandler {
    public typealias InboundIn = RPCResponse
    public typealias OutboundIn = RPCRequestWrapper
    public typealias OutboundOut = RPCRequest

    private var queue = CircularBuffer<(id: UUID, promise: EventLoopPromise<RPCResponse>)>()

    func write(context: ChannelHandlerContext, data: NIOAny, promise: EventLoopPromise<Void>?) {
        let requestWrapper = self.unwrapOutboundIn(data)
        queue.append((requestWrapper.id,requestWrapper.promise))
        context.write(wrapOutboundOut(requestWrapper.request), promise: promise)
    }

    public func channelRead(context: ChannelHandlerContext, data: NIOAny) {
        if self.queue.isEmpty {
            return context.fireChannelRead(data)  // already complete
        }

        let promise = queue.removeFirst().promise
        let response = unwrapInboundIn(data)
        promise.succeed(response)
    }

    public func errorCaught(context: ChannelHandlerContext, error: Error) {
        if let remoteAddress = context.remoteAddress {
            logger.debug("server \(remoteAddress) error \(error)")
        }
        if self.queue.isEmpty {
            return context.fireErrorCaught(error) // already complete
        }
        let item = queue.removeFirst()
        let promise = item.promise
        promise.fail(error)
            // close the connection
        context.close(promise: nil)
    }

    public func channelActive(context: ChannelHandlerContext) {
        if let remoteAddress = context.remoteAddress {
            logger.debug("server \(remoteAddress) connected")
        }
    }

    public func channelInactive(context: ChannelHandlerContext) {
        if let remoteAddress = context.remoteAddress {
            logger.debug("server \(remoteAddress) disconnected")
        }
        if !self.queue.isEmpty {
            self.errorCaught(context: context, error: ClientError.connectionResetByPeer)
        }
    }

    func userInboundEventTriggered(context: ChannelHandlerContext, event: Any) {
        if (event as? IdleStateHandler.IdleStateEvent) == .read {
            self.errorCaught(context: context, error: ClientError.timeout)
        } else {
            context.fireUserInboundEventTriggered(event)
        }
    }

}

struct XMLRPCClient {
    static let requestBegin = "<?xml version=\"1.0\"?>\r\n<methodCall>\r\n<methodName>"
    static let requestEnd = "</methodCall>\r\n"
    static let methodresponseTag = "<methodResponse>"


        static func parseResponse(xml: String) -> XmlRpcValue? {
            guard let tagIndex = xml.range(of: methodresponseTag) else {
                logger.error("Invalid response - no methodResponse. Response:\n\(xml)")
                return nil
            }

            var result = XmlRpcValue()

            var data = xml.suffix(from: tagIndex.upperBound)
            if XmlRpcUtil.nextTagIs(tag: .params, xml: &data) && XmlRpcUtil.nextTagIs(tag: .param, xml: &data) {
                if result.fromXML(xml: &data) {
                    return result
                } else {
                    logger.error("Invalid response value. Response: \(data)")
                }
            } else if XmlRpcUtil.nextTagIs(tag: .fault, xml: &data) {
                logger.error("Invalid response value. Response: \(data)")
            } else {
                logger.error("Invalid response - no param or fault tag. Response \(xml)")
            }

            return nil
        }

    static func generateHeader(body: String, host: String, port: UInt16) -> String {
        let header = "POST "
            + "/"
            + " HTTP/1.1\r\nUser-Agent: "
            + xmlrpcVersion
            + "\r\nHost: "
            + host
            + ":\(port)\r\n"
            + "Content-Type: text/xml\r\nContent-length: "
            + "\(body.lengthOfBytes(using: .utf8))\r\n\r\n"
        return header
    }

    static func generateRequest(methodName: String, params: XmlRpcValue, host: String, port: UInt16) -> String {
        var body = XMLRPCClient.requestBegin + methodName + Tags.endMethodname.rawValue

        // If params is an array, each element is a separate parameter
        if params.valid() {
            body += Tags.params.rawValue
            if case .array(let array) = params {
                for a in array {
                    body += Tags.param.rawValue
                    body += a.toXml()
                    body += Tags.endParam.rawValue
                }
            } else {
                body += Tags.param.rawValue
                body += params.toXml()
                body += Tags.endParam.rawValue
            }

            body += Tags.endParams.rawValue
        }
        body += XMLRPCClient.requestEnd

        let header = generateHeader(body: body, host: host, port: port)
        let request = header + body
        return request
    }


    }



    public final class Master {
        let group: EventLoopGroup
        var channel: Channel?

        public init(group: EventLoopGroup) {
            self.group = group
        }

        public func connect(host: String, port: Int) -> EventLoopFuture<Master> {
            assert(.initializing == self.state)

            

            let bootstrap = ClientBootstrap(group: self.group)
                // Enable SO_REUSEADDR.
                .channelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
                .channelInitializer { channel in
                    channel.pipeline.addHandlers([ByteToMessageHandler(XmlRpcMessageDelimiterCodec()),
                                                  RPCCodec(),
                                                  XmlRpcHandler()])
            }
            self.state = .connecting("\(host):\(port)")

            return bootstrap.connect(host: host, port: port).flatMap { channel in
                self.channel = channel
                self.state = .connected
                return channel.eventLoop.makeSucceededFuture(self)
            }
        }

        public func disconnect() -> EventLoopFuture<Void> {
            if .connected != self.state {
                return self.group.next().makeFailedFuture(ClientError.notReady)
            }
            guard let channel = self.channel else {
                return self.group.next().makeFailedFuture(ClientError.notReady)
            }
            self.state = .disconnecting
            channel.closeFuture.whenComplete { _ in
                self.state = .disconnected
            }
            channel.close(promise: nil)
            return channel.closeFuture
        }


        func validateXmlrpcResponse(method: String, response: XmlRpcValue) -> XmlRpcValue? {

            guard response.isArray else {
                logger.debug("XML-RPC call [\(method)] didn't return an array")
                return nil
            }

            var resp = response
            if resp.size() == 1 {
                resp = resp[0]
            }

            if resp.size() != 2 && resp.size() != 3 {
                logger.debug("XML-RPC call [\(method)] didn't return a 2 or 3-element array")
                return nil
            }
            guard case .int(let statusCode) = resp[0] else {
                logger.debug("XML-RPC call [\(method)] didn't return a int as the 1st element")
                return nil
            }
            guard case .string(let statusString) = resp[1] else {
                logger.debug("XML-RPC call [\(method)] didn't return a string as the 2nd element")
                return nil
            }
            if statusCode != 1 {
                logger.debug("XML-RPC call [\(method)] returned an error (\(statusCode): [\(statusString)]")
                return nil
            }
            if resp.size() > 2 {
                if case .string(let uri) = resp[2] {
                    return XmlRpcValue(anyArray: [uri])
                } else {
                    return resp[2]
                }
            }
            return XmlRpcValue(anyArray: [])
        }

        enum MasterError: Error {
            case invalidResponse(String)
        }

        public typealias ClientResult = Result<XmlRpcValue,ClientError>

        public func send(method: String, request: XmlRpcValue) -> EventLoopFuture<ClientResult> {
            if .connected != self.state {
                return self.group.next().makeFailedFuture(ClientError.notReady)
            }
            guard let channel = self.channel else {
                return self.group.next().makeFailedFuture(ClientError.notReady)
            }

            let promise: EventLoopPromise<RPCResponse> = channel.eventLoop.makePromise()
            let req = RPCRequest(methodName: method, params: request)
            let wrapper = RPCRequestWrapper(request: req, promise: promise)
            let future = channel.writeAndFlush(wrapper)
            future.cascadeFailure(to: promise) // if write fails
            return future.flatMap { _ in
                promise.futureResult.map {
                    .success($0.result)
                }
            }

        }

        private var _state = State.initializing
        private let lock = NIOLock()
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
            case connecting(String)
            case connected
            case disconnecting
            case disconnected
        }

        public enum ClientError: Error {
            case notReady
            case cantBind
            case timeout
            case connectionResetByPeer
        }



    }

}


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
