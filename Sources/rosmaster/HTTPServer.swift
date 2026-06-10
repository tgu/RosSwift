//
//  HTTPServer.swift
//  rosmaster
//
//  Created by Thomas Gustafsson on 2019-04-06.
//

import Foundation
import Atomics
import NIO
import NIOCore
import NIOConcurrencyHelpers
import NIOExtras
import NIOHTTP1
import Logging
import rpcobject
import Synchronization

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

/// Closes the channel on receipt of `ChannelShouldQuiesceEvent`, allowing pending
/// writes to flush as part of NIO's normal close sequence. Installed in each
/// child pipeline so that `ServerQuiescingHelper.initiateShutdown` causes
/// active connections to terminate gracefully.
private final class QuiesceCloseHandler: ChannelInboundHandler, RemovableChannelHandler, Sendable {
    typealias InboundIn = NIOAny

    func userInboundEventTriggered(context: ChannelHandlerContext, event: Any) {
        if event is ChannelShouldQuiesceEvent {
            context.close(promise: nil)
            return
        }
        context.fireUserInboundEventTriggered(event)
    }
}

public final class XMLRPCServer: Sendable {
    private static let maxBodyBytes = 8 * 1024 * 1024  // 8 MiB

    typealias HTTPChannel = NIOAsyncChannel<HTTPServerRequestPart, HTTPServerResponsePart>
    typealias ServerChannel = NIOAsyncChannel<HTTPChannel, Never>

    private let group: EventLoopGroup
    private let closure: @Sendable (String, [XmlRpcValue]) async -> XmlRpcValue
    private let quiesce: ServerQuiescingHelper

    private struct State: Sendable {
        enum Phase: Sendable {
            case initializing
            case starting(String)
            case started
            case stopping
            case stopped
        }
        var phase: Phase = .initializing
        var serveTask: Task<Void, Never>?
    }
    private let state = Mutex(State())
    private let _boundPort = ManagedAtomic<Int32>(0)

    /// The port the listener actually bound to. When `start(host:port:)` was
    /// called with `port: 0` (let the OS pick), this is the assigned port;
    /// otherwise it matches the requested port. Returns 0 before bind.
    public var boundPort: Int { Int(_boundPort.load(ordering: .relaxed)) }

    init(group: EventLoopGroup, handler: @escaping @Sendable (String, [XmlRpcValue]) async -> XmlRpcValue) {
        self.group = group
        self.closure = handler
        self.quiesce = ServerQuiescingHelper(group: group)
    }

    func start(host: String, port: Int) -> EventLoopFuture<XMLRPCServer> {
        let proceed: Bool = state.withLock {
            if case .initializing = $0.phase {
                $0.phase = .starting("\(host):\(port)")
                logger.debug("XMLRPCServer state -> starting(\(host):\(port))")
                return true
            }
            return false
        }
        guard proceed else {
            return group.next().makeFailedFuture(ServerError.notReady)
        }

        let promise = group.next().makePromise(of: XMLRPCServer.self)
        Task {
            do {
                let serverChannel = try await self.bind(host: host, port: port)
                if let p = serverChannel.channel.localAddress?.port {
                    self._boundPort.store(Int32(p), ordering: .relaxed)
                }
                self.state.withLock {
                    $0.phase = .started
                    $0.serveTask = Task { [weak self] in
                        await self?.serve(serverChannel: serverChannel)
                    }
                }
                logger.debug("XMLRPCServer state -> started on port \(self.boundPort)")
                promise.succeed(self)
            } catch {
                self.state.withLock {
                    $0.phase = .stopped
                }
                logger.error("XMLRPCServer bind failed: \(error)")
                promise.fail(error)
            }
        }
        return promise.futureResult
    }

    func stop() -> EventLoopFuture<Void> {
        let serveTask: Task<Void, Never>? = state.withLock {
            guard case .started = $0.phase else { return nil }
            $0.phase = .stopping
            let task = $0.serveTask
            $0.serveTask = nil
            return task
        }
        guard let serveTask else {
            return group.next().makeFailedFuture(ServerError.notReady)
        }

        let promise = group.next().makePromise(of: Void.self)
        Task {
            // Graceful shutdown: close listener and signal each open child connection
            // via ChannelShouldQuiesceEvent. QuiesceCloseHandler converts that to a
            // channel close, which lets NIO flush any pending writes first.
            let quiescePromise = self.group.next().makePromise(of: Void.self)
            self.quiesce.initiateShutdown(promise: quiescePromise)
            _ = try? await quiescePromise.futureResult.get()
            // Drain in-flight handler tasks (they finish when their writes fail or complete).
            await serveTask.value
            self.state.withLock { $0.phase = .stopped }
            logger.debug("XMLRPCServer state -> stopped")
            promise.succeed(())
        }
        return promise.futureResult
    }

    // MARK: - Bind & accept loop

    private func bind(host: String, port: Int) async throws -> ServerChannel {
        let quiesce = self.quiesce
        return try await ServerBootstrap(group: group)
            .serverChannelOption(ChannelOptions.backlog, value: 256)
            .serverChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
            .serverChannelInitializer { serverChannel in
                serverChannel.eventLoop.makeCompletedFuture {
                    try serverChannel.pipeline.syncOperations.addHandler(
                        quiesce.makeServerChannelHandler(channel: serverChannel)
                    )
                }
            }
            .childChannelOption(ChannelOptions.socket(IPPROTO_TCP, TCP_NODELAY), value: 1)
            .childChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
            .childChannelOption(ChannelOptions.maxMessagesPerRead, value: 1)
            .childChannelOption(ChannelOptions.allowRemoteHalfClosure, value: true)
            .bind(host: host, port: port) { childChannel in
                childChannel.eventLoop.makeCompletedFuture {
                    try childChannel.pipeline.syncOperations.configureHTTPServerPipeline(
                        withPipeliningAssistance: true,
                        withErrorHandling: true
                    )
                    try childChannel.pipeline.syncOperations.addHandler(QuiesceCloseHandler())
                    return try HTTPChannel(
                        wrappingChannelSynchronously: childChannel,
                        configuration: .init()
                    )
                }
            }
    }

    private func serve(serverChannel: ServerChannel) async {
        do {
            try await serverChannel.executeThenClose { inbound in
                try await withThrowingDiscardingTaskGroup { group in
                    for try await connection in inbound {
                        group.addTask {
                            await self.handle(connection: connection)
                        }
                    }
                }
            }
        } catch is CancellationError {
            // Expected during stop()
        } catch {
            logger.debug("XMLRPCServer accept loop error: \(error)")
        }
    }

    // MARK: - Per-connection request/response loop

    private func handle(connection: HTTPChannel) async {
        do {
            try await connection.executeThenClose { inbound, outbound in
                var iterator = inbound.makeAsyncIterator()
                while let part = try await iterator.next() {
                    guard case .head(let head) = part else {
                        // protocol error: expected .head
                        return
                    }
                    let keepAlive = head.isKeepAlive
                    var buffer = ByteBuffer()

                    bodyLoop: while let bodyPart = try await iterator.next() {
                        switch bodyPart {
                        case .head:
                            return  // protocol error
                        case .body(var buf):
                            if buffer.readableBytes + buf.readableBytes > Self.maxBodyBytes {
                                logger.warning("request body exceeded \(Self.maxBodyBytes) bytes; rejecting")
                                try await self.writeError(outbound: outbound, requestHead: head, status: .payloadTooLarge)
                                return
                            }
                            buffer.writeBuffer(&buf)
                        case .end:
                            break bodyLoop
                        }
                    }

                    guard let str = buffer.readString(length: buffer.readableBytes) else {
                        logger.warning("failed to read request body as string; responding 400")
                        try await self.writeError(outbound: outbound, requestHead: head, status: .badRequest)
                        return
                    }
                    let obj = XmlRpcUtil.parseRequest(xml: str)
                    guard !obj.method.isEmpty else {
                        logger.warning("could not parse XML-RPC method from request; responding 400")
                        try await self.writeError(outbound: outbound, requestHead: head, status: .badRequest)
                        return
                    }

                    let resp = await self.executeRequest(method: obj.method, params: obj.params)
                    let responseXML = self.generateResponse(resultXML: resp.toXml())

                    var responseBuffer = ByteBuffer()
                    responseBuffer.writeString(responseXML)

                    var responseHead = httpResponseHead(request: head, status: .ok)
                    responseHead.headers.add(name: "content-length", value: "\(responseBuffer.readableBytes)")
                    responseHead.headers.add(name: "content-type", value: "text/xml")

                    try await outbound.write(.head(responseHead))
                    try await outbound.write(.body(.byteBuffer(responseBuffer)))
                    try await outbound.write(.end(nil))

                    if !keepAlive { return }
                }
            }
        } catch is CancellationError {
            // Server stopping or peer closed mid-read
        } catch {
            logger.debug("XMLRPCServer connection error: \(error)")
        }
    }

    private func writeError(
        outbound: NIOAsyncChannelOutboundWriter<HTTPServerResponsePart>,
        requestHead: HTTPRequestHead,
        status: HTTPResponseStatus
    ) async throws {
        var head = httpResponseHead(request: requestHead, status: status)
        head.headers.replaceOrAdd(name: "content-length", value: "0")
        head.headers.replaceOrAdd(name: "connection", value: "close")
        try await outbound.write(.head(head))
        try await outbound.write(.end(nil))
    }

    // MARK: - XML-RPC dispatch

    private func executeRequest(method: String, params: [XmlRpcValue]) async -> XmlRpcValue {
        if method == "system.multicall" {
            precondition(params.count == 1)
            let par = params[0]
            let calls = par.size()
            var res = [XmlRpcValue]()
            for i in 0..<calls {
                let x = par[i]
                guard let methodDict = x.dictionary,
                      let methodName = methodDict["methodName"]?.string,
                      let parameters = methodDict["params"]?.array else {
                    return XmlRpcValue(anyArray: [-1, "", 0])
                }
                let r = await closure(methodName, parameters)
                res.append(r)
            }
            return XmlRpcValue(anyArray: [1, "", res])
        } else {
            return await closure(method, params)
        }
    }

    private func generateResponse(resultXML: String) -> String {
        let res = "<?xml version=\"1.0\"?>\r\n<methodResponse><params><param>"
        let req = "</param></params></methodResponse>\r\n"
        return res + resultXML + req
    }
}


internal enum ServerError: Error {
    case notReady
    case cantBind
    case timeout
}
