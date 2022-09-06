//
//  ConnectionManager.swift
//
//  Created by Thomas Gustafsson on 2018-03-22.
//

import BinaryCoder
import NIO
import Atomics
import StdMsgs

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
                let link = ServiceClientLink(connection: conn)
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
                if let response = serviceLink.parent?.processRequest(buf: rawMessage) {
                    let rawResponse = SerializedMessage(msg: response)
                    var buf = context.channel.allocator.buffer(capacity: rawResponse.buf.count + 1)
                    buf.writeBytes([UInt8(1)]+rawResponse.buf)  // Start with Bool(true)
                    let niodata = self.wrapInboundOut(buf)

                    context.writeAndFlush(niodata, promise: nil)
                } else {
                    ROS_WARNING("ServiceCall not able process")
                    context.close(promise: nil)
                }
            }
        }

    }

    func errorCaught(context: ChannelHandlerContext, error: Error) {
        ROS_ERROR(error.localizedDescription)
    }

}

internal final class ConnectionManager {
    var channel: Channel?
    var boot: ServerBootstrap?
    unowned var ros: Ros!

    internal init() {
    }

    var port: Int32 {
        return Int32(channel?.localAddress?.port ?? 0)
    }

    func clear(reason: DropReason) {
        fatalError()
    }

    func shutdown() {
        // Nothing to do here
    }

    // The connection ID counter, used to assign unique ID to each inbound or
    // outbound connection.  Access via getNewConnectionID()
    private var connectionIdCounter = ManagedAtomic<Int>(0)

    func getNewConnectionID() -> Int {
        return connectionIdCounter.loadThenWrappingIncrement(ordering: .relaxed)
    }

    func start(ros: Ros) {
        self.ros = ros
        initialize(group: threadGroup)
        do {
            channel = try boot?.bind(host: ros.network.gHost, port: 0).wait()
            ROS_DEBUG("listening channel on port [\(channel?.localAddress?.host ?? "unknown host"):\(port)]")
        } catch {
            ROS_ERROR("Could not bind to [\(port)]: \(error)")
        }
    }

    private func initialize(group: EventLoopGroup) {

        boot = ServerBootstrap(group: group)
            // Specify backlog and enable SO_REUSEADDR for the server itself
            .serverChannelOption(ChannelOptions.backlog, value: 256)
            .serverChannelOption(ChannelOptions.autoRead, value: true)
            .serverChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_KEEPALIVE), value: 1)

            // Set the handlers that are appled to the accepted Channels
            .childChannelInitializer {
                $0.pipeline.addHandlers([ByteToMessageHandler(MessageDelimiterCodec()),
                                         ConnectionHandler(ros: self.ros)])
            }

            // Enable TCP_NODELAY and SO_REUSEADDR for the accepted Channels
            .childChannelOption(ChannelOptions.socket(IPPROTO_TCP, TCP_NODELAY), value: 1)
            .childChannelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
            .childChannelOption(ChannelOptions.maxMessagesPerRead, value: 16)
            .childChannelOption(ChannelOptions.recvAllocator, value: AdaptiveRecvByteBufferAllocator())
    }

}
