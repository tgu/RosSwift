//
//  XmlRpcServer.swift
//  swiftros
//
//  Created by Thomas Gustafsson on 2018-03-22.
//

import BinaryCoder
import Foundation
import NIO
import NIOConcurrencyHelpers
import StdMsgs

final class ConnectionHandler: ChannelInboundHandler {
    typealias InboundIn = ByteBuffer
    typealias InboundOut = ByteBuffer

    private var subscriber: TransportSubscriberLink?
    private var serviceclient: ServiceClientLink?

    private let ros: Ros

    var header = Header()

    func channelInactive(context: ChannelHandlerContext) {
        subscriber?.drop()
        serviceclient?.drop()
        //        ROS_DEBUG("ConnectionHandler channelInactive for port \(context.channel.localAddress!.port!)")
        context.fireChannelInactive()
    }

    init(ros: Ros) {
        self.ros = ros
    }

    enum State {
        case header
        case serviceCall
    }

    var state = State.header

    public func channelRead(context: ChannelHandlerContext, data: NIOAny) {
        //        ROS_DEBUG("message from \(context.channel.remoteAddress) => \(context.channel.localAddress)")
        var buffer = self.unwrapInboundIn(data)

        switch state {
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

            header.headers = readMap

            // Connection Header Received

            if let topic = readMap["topic"], let remote = context.remoteAddress {
                let conn = Connection(transport: context.channel, header: header)
                ROS_DEBUG("Connection: Creating TransportSubscriberLink for topic [\(topic)] connected to [\(remote)]")
                let subLink = TransportSubscriberLink(connection: conn, topicManager: ros.topicManager)
                if subLink.handleHeader(ros: ros, header: header) {
                    subscriber = subLink
                } else {
                    subLink.drop()
                }
            } else if let val = header.getValue(key: "service") {
                ROS_DEBUG("Connection: Creating ServiceClientLink for service [\(val)] connected to [\(String(describing: context.remoteAddress!.description))]")
                let conn = Connection(transport: context.channel, header: header)

                let link = ServiceClientLink()
                link.initialize(connection: conn)
                if link.handleHeader(header: header, ros: ros) {
                    serviceclient = link
                }
                state = .serviceCall
            } else {
                ROS_ERROR("Got a connection for a type other than 'topic' or 'service' from [\(String(describing: context.remoteAddress?.description))].  Fail.")
                fatalError()
            }

        case .serviceCall:
            if let rawMessage = buffer.readBytes(length: buffer.readableBytes) {
                guard let link = serviceclient else {
                    ROS_ERROR("ServiceCall not able process: link is missing")
                    context.close(promise: nil)
                    return
                }

                if let response = link.parent?.processRequest(buf: rawMessage) {
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

    func getTCPPort() -> Int32 {
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
    private var connectionIdCounter = Atomic<Int>(value: 0)

    func getNewConnectionID() -> Int {
        return connectionIdCounter.add(1)
    }

    func start(ros: Ros) {
        self.ros = ros
        initialize(group: threadGroup)
        do {
            channel = try boot?.bind(host: ros.network.getHost(), port: 0).wait()
            ROS_DEBUG("listening channel on port [\(channel?.localAddress?.host ?? "unknown host"):\(getTCPPort())]")

        } catch {
            ROS_ERROR("Could not bind to [\(getTCPPort())]: \(error)")
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
