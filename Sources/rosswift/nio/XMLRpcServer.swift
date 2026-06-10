//
//  XmlRpcServer.swift
//  swiftros
//
//  Created by Thomas Gustafsson on 2018-03-22.
//

import Foundation
import Atomics
import NIO
import NIOCore
import NIOConcurrencyHelpers
import NIOExtras
import NIOHTTP1
import rpcobject
import Synchronization

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
            head.headers.add(name: "Connection", value: "keep-alive")
        case (false, 1, let n) where n >= 1:
            head.headers.add(name: "Connection", value: "close")
        default:
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

final class XMLRPCServer: Sendable {
    private static let maxBodyBytes = 8 * 1024 * 1024  // 8 MiB

    typealias HTTPChannel = NIOAsyncChannel<HTTPServerRequestPart, HTTPServerResponsePart>
    typealias ServerChannel = NIOAsyncChannel<HTTPChannel, Never>

    private let group: EventLoopGroup
    private let methods = Mutex<[String: XMLRPCFunc]>([:])
    private let quiesce: ServerQuiescingHelper

    private struct State: Sendable {
        enum Phase: Sendable {
            case initializing
            case bound
            case stopping
            case stopped
        }
        var phase: Phase = .initializing
        var serveTask: Task<Void, Never>?
    }
    private let state = Mutex(State())
    private let _serverPort = ManagedAtomic<Int32>(0)

    var serverPort: Int32 { _serverPort.load(ordering: .relaxed) }

    init(group: EventLoopGroup) {
        self.group = group
        self.quiesce = ServerQuiescingHelper(group: group)
    }

    // MARK: - Method registry

    func add(method: @escaping @Sendable XMLRPCFunc, named: String) -> Bool {
        methods.withLock { m in
            if m[named] != nil {
                ROS_ERROR("method '\(named)' already bound\n\tregistered methods: \(m)")
                return false
            }
            m[named] = method
            return true
        }
    }

    func removeMethod(method: XmlRpcServerMethod) {
        remove(methodName: method.name)
    }

    func remove(methodName: String) {
        methods.withLock { _ = $0.removeValue(forKey: methodName) }
    }

    func removeAll() {
        methods.withLock { $0.removeAll() }
    }

    func find(method name: String) -> XMLRPCFunc? {
        methods.withLock { $0[name] }
    }

    // MARK: - Bind & serve

    func bindAndListen(host: String, port: Int) async {
        let proceed: Bool = state.withLock {
            if case .initializing = $0.phase {
                return true
            }
            return false
        }
        guard proceed else {
            ROS_ERROR("XMLRPCServer.bindAndListen called twice")
            return
        }

        do {
            let serverChannel = try await bind(host: host, port: port)
            if let p = serverChannel.channel.localAddress?.port {
                _serverPort.store(Int32(p), ordering: .relaxed)
            }
            state.withLock {
                $0.phase = .bound
                $0.serveTask = Task { [weak self] in
                    await self?.serve(serverChannel: serverChannel)
                }
            }
            ROS_DEBUG("Service up and running on \(host):\(serverPort)")
        } catch {
            state.withLock { $0.phase = .stopped }
            ROS_ERROR("bind failed to [\(host)], \(error)")
        }
    }

    /// Gracefully shuts down the listener, signaling all child connections to
    /// quiesce, and waits for in-flight handlers to drain.
    func shutdown() async {
        let serveTask: Task<Void, Never>? = state.withLock {
            guard case .bound = $0.phase else { return nil }
            $0.phase = .stopping
            let task = $0.serveTask
            $0.serveTask = nil
            return task
        }
        guard let serveTask else { return }

        let quiescePromise = group.next().makePromise(of: Void.self)
        quiesce.initiateShutdown(promise: quiescePromise)
        _ = try? await quiescePromise.futureResult.get()
        await serveTask.value
        state.withLock { $0.phase = .stopped }
        ROS_DEBUG("XMLRPCServer stopped")
    }

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
            // Expected during shutdown
        } catch {
            ROS_DEBUG("XMLRPCServer accept loop error: \(error)")
        }
    }

    // MARK: - Per-connection request loop

    private func handle(connection: HTTPChannel) async {
        do {
            try await connection.executeThenClose { inbound, outbound in
                var iterator = inbound.makeAsyncIterator()
                while let part = try await iterator.next() {
                    guard case .head(let head) = part else { return }
                    let keepAlive = head.isKeepAlive
                    var buffer = ByteBuffer()

                    bodyLoop: while let bodyPart = try await iterator.next() {
                        switch bodyPart {
                        case .head:
                            return  // protocol error
                        case .body(var buf):
                            if buffer.readableBytes + buf.readableBytes > Self.maxBodyBytes {
                                ROS_DEBUG("XMLRPCServer request body exceeded \(Self.maxBodyBytes) bytes; rejecting")
                                try await self.writeError(outbound: outbound, requestHead: head, status: .payloadTooLarge)
                                return
                            }
                            buffer.writeBuffer(&buf)
                        case .end:
                            break bodyLoop
                        }
                    }

                    guard let str = buffer.readString(length: buffer.readableBytes) else {
                        ROS_DEBUG("XMLRPCServer failed to read request body as string; responding 400")
                        try await self.writeError(outbound: outbound, requestHead: head, status: .badRequest)
                        return
                    }
                    let obj = XmlRpcUtil.parseRequest(xml: str)
                    guard !obj.method.isEmpty else {
                        ROS_DEBUG("XMLRPCServer could not parse XML-RPC method from request; responding 400")
                        try await self.writeError(outbound: outbound, requestHead: head, status: .badRequest)
                        return
                    }

                    let params = XmlRpcValue(anyArray: obj.params)
                    let resp = self.executeMethod(methodName: obj.method, params: params)
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
            ROS_DEBUG("XMLRPCServer connection error: \(error)")
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

    private func executeMethod(methodName: String, params: XmlRpcValue) -> XmlRpcValue {
        if let method = find(method: methodName) {
            return method(params)
        }
        return XmlRpcValue(str: "")
    }

    private func generateResponse(resultXML: String) -> String {
        let res = "<?xml version=\"1.0\"?>\r\n<methodResponse><params><param>"
        let req = "</param></params></methodResponse>\r\n"
        return res + resultXML + req
    }
}
