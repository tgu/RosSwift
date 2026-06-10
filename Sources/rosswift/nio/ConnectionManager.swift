//
//  ConnectionManager.swift
//
//  Created by Thomas Gustafsson on 2018-03-22.
//

import BinaryCoder
import NIO
import NIOConcurrencyHelpers
import Atomics
import StdMsgs
import Foundation
import Synchronization

final class ConnectionHandler: ChannelInboundHandler {
    typealias InboundIn = ByteBuffer
    typealias InboundOut = ByteBuffer

    private let ros: Ros

    func channelInactive(context: ChannelHandlerContext) {
        switch state {
        case .serviceCall(let serviceLink):
            serviceLink.dropServiceClient()
        case .subscriber(let subscriber):
       	 	subscriber.dropParentPublication()
        default:
            break
        }
        state = .inactive
        context.fireChannelInactive()
    }

    init(ros: Ros) {
        self.ros = ros
    }

    enum ConnectionState {
        case inactive
        case header
        case subscriber(TransportSubscriberLink)
        case serviceCall(ServiceClientLink)
    }

    var state = ConnectionState.header

    func channelRead(context: ChannelHandlerContext, data: NIOAny) {
        var buffer = self.unwrapInboundIn(data)

        switch state {
        case .inactive:
            ROS_ERROR("Channel is inactive")
            context.close(promise: nil)
        case .header:
            guard let len: UInt32 = buffer.readInteger(endianness: .little) else {
                fatalError()
            }
            precondition(len <= buffer.readableBytes)
            let leave = buffer.readableBytes - Int(len)
            var readMap = [String: String]()

            while buffer.readableBytes > leave {
                guard let topicLen: UInt32 = buffer.readInteger(endianness: .little) else {
                    ROS_DEBUG("Received an invalid TCPROS header.  invalid string")
                    return
                }

                guard let line = buffer.readString(length: Int(topicLen)) else {
                    ROS_DEBUG("Received an invalid TCPROS header.  Each line must have an equals sign.")
                    return
                }

                guard let eq = line.firstIndex(of: "=") else {
                    ROS_DEBUG("Received an invalid TCPROS header.  Each line must have an equals sign.")
                    return
                }
                let key = String(line.prefix(upTo: eq))
                let value = String(line.suffix(from: eq).dropFirst())
                readMap[key] = value
            }

            ROS_DEBUG(readMap.description)

            if let error = readMap["error"] {
                ROS_ERROR("Received error message in header for connection to [\(context.remoteAddress?.description ?? "empty")]]: [\(error)]")
                return
            }

            let header = Header(headers: readMap)

            // Connection Header Received

            if let topic = readMap["topic"], let remote = context.remoteAddress {
                let conn = Connection(transport: context.channel, header: header)
                ROS_DEBUG("Connection: Creating TransportSubscriberLink for topic [\(topic)] connected to [\(remote)]")
                if let subLink = TransportSubscriberLink(ros: ros, connection: conn, header: header) {
                    state = .subscriber(subLink)
                }
            } else if let val = header["service"] {
                ROS_DEBUG("Connection: Creating ServiceClientLink for service [\(val)] connected to [\(String(describing: context.remoteAddress!.description))]")
                let conn = Connection(transport: context.channel, header: header)
                var link = ServiceClientLink(connection: conn)
                if link.handleHeader(header: header, ros: ros) {
                    state = .serviceCall(link)
                }
            } else {
                ROS_ERROR("Got a connection for a type other than 'topic' or 'service' from [\(String(describing: context.remoteAddress?.description))].  Fail.")
                context.close(promise: nil)
            }
            
        case .subscriber:
            ROS_ERROR("Got a unexpected connection [\(String(describing: context.remoteAddress?.description))].")
            context.close(promise: nil)

        case .serviceCall(let serviceLink):
            if let rawMessage = buffer.readBytes(length: buffer.readableBytes) {
                guard let parent = serviceLink.parent else {
                    ROS_WARNING("ServiceCall received after publication was dropped")
                    context.close(promise: nil)
                    break
                }
                switch parent.processRequest(buf: rawMessage) {
                case .success(let response):
                    let rawResponse = SerializedMessage(msg: response)
                    var buf = context.channel.allocator.buffer(capacity: rawResponse.buf.count + 1)
                    buf.writeBytes([UInt8(1)] + rawResponse.buf)  // Start with Bool(true)
                    let niodata = self.wrapInboundOut(buf)
                    context.writeAndFlush(niodata, promise: nil)
                case .failure(let errMsg):
                    // Per TCPROS spec, signal failure with a 0 byte followed
                    // by a length-prefixed error string so the client doesn't
                    // hang waiting for a response.
                    ROS_WARNING("ServiceCall failed: \(errMsg)")
                    let errBytes = Array(errMsg.utf8)
                    var buf = context.channel.allocator.buffer(capacity: errBytes.count + 5)
                    buf.writeBytes([UInt8(0)])
                    buf.writeInteger(UInt32(errBytes.count), endianness: Endianness.little)
                    buf.writeBytes(errBytes)
                    let niodata = self.wrapInboundOut(buf)
                    context.writeAndFlush(niodata, promise: nil)
                    context.close(promise: nil)
                }
            }
        }

    }

    func errorCaught(context: ChannelHandlerContext, error: Error) {
        ROS_ERROR(error.localizedDescription)
    }

}

internal final class ConnectionManager: RosManager, @unchecked Sendable {
    let rosID: RosID

    // Listener channel + active child connections, kept under a lock so
    // `clear` and `shutdown` can safely iterate and close. NIO's `Channel`
    // isn't unconditionally `Sendable`, so we wrap the storage with a
    // NIOLock and mark the class `@unchecked Sendable` — the lock provides
    // the actual safety.
    private let lock = NIOLock()
    private var _listener: Channel?
    private var _activeChildren: [ObjectIdentifier: Channel] = [:]
    private let _port = ManagedAtomic<Int32>(0)

    // The connection ID counter, used to assign unique ID to each inbound or
    // outbound connection.  Access via getNewConnectionID()
    private let connectionIdCounter = ManagedAtomic<Int>(0)

    var ros: Ros {
        guard let r = Ros.getGlobalRos(for: rosID) else {
            fatalError()
        }
        return r
    }

    init(rosID: RosID) {
        self.rosID = rosID
    }

    var port: Int32 { _port.load(ordering: .relaxed) }

    /// Closes all open child (transport) connections. Used by the
    /// `~debug/close_all_connections` XML-RPC service and on shutdown.
    /// Does NOT close the listener — new connections can still be accepted.
    func clear(reason: DropReason) {
        let children: [Channel] = lock.withLock { Array(_activeChildren.values) }
        if !children.isEmpty {
            ROS_DEBUG("ConnectionManager.clear(\(reason)) — closing \(children.count) connection(s)")
        }
        for ch in children {
            ch.close(promise: nil)
        }
    }

    /// Closes all child connections and the listener.
    func shutdown() {
        clear(reason: .destructing)
        let listener: Channel? = lock.withLock {
            let l = _listener
            _listener = nil
            return l
        }
        listener?.close(promise: nil)
    }

    func getNewConnectionID() -> UUID {
        UUID()
        //return connectionIdCounter.loadThenWrappingIncrement(ordering: .relaxed)
    }

    func start() async {
        let bootstrap = makeBootstrap(group: threadGroup)
        do {
            let channel = try await bootstrap.bind(host: ros.network.gHost, port: 0).get()
            lock.withLock { _listener = channel }
            if let p = channel.localAddress?.port {
                _port.store(Int32(p), ordering: .relaxed)
            }
            ROS_DEBUG("listening channel on port [\(channel.localAddress?.host ?? "unknown host"):\(port)]")
        } catch {
            ROS_ERROR("Could not bind: \(error)")
        }
    }

    private func registerChild(_ channel: Channel) {
        let key = ObjectIdentifier(channel)
        lock.withLock { _activeChildren[key] = channel }
        channel.closeFuture.whenComplete { [weak self] _ in
            self?.lock.withLock { _ = self?._activeChildren.removeValue(forKey: key) }
        }
    }

    private func makeBootstrap(group: EventLoopGroup) -> ServerBootstrap {
        ServerBootstrap(group: group)
            // Specify backlog and enable SO_REUSEADDR for the server itself
            .serverChannelOption(ChannelOptions.backlog, value: 256)
            .serverChannelOption(ChannelOptions.autoRead, value: true)
            .serverChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_KEEPALIVE), value: 1)

            // Set the handlers that are appled to the accepted Channels
            .childChannelInitializer { channel in
                self.registerChild(channel)
                return channel.eventLoop.makeCompletedFuture {
                    try channel.pipeline.syncOperations.addHandlers([ByteToMessageHandler(MessageDelimiterCodec()),
                                                  ConnectionHandler(ros: self.ros)])
                }
            }

            // Enable TCP_NODELAY and SO_REUSEADDR for the accepted Channels
            .childChannelOption(ChannelOptions.socket(IPPROTO_TCP, TCP_NODELAY), value: 1)
            .childChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
            .childChannelOption(ChannelOptions.maxMessagesPerRead, value: 16)
            .childChannelOption(ChannelOptions.recvAllocator, value: AdaptiveRecvByteBufferAllocator())
    }

}
