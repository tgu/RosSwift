//
//  inboundConnection.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-23.
//

import BinaryCoder
import Foundation
import NIO
import NIOConcurrencyHelpers
import StdMsgs

enum ConnectionError: Error {
    case connectionDropped
}

final class InboundConnection: ConnectionProtocol {

    var channel: Channel?
    var dropped = Atomic<Bool>(value: false)
    weak var parent: Subscription?
    weak var link: TransportPublisherLink?
    let host: String
    let port: Int
    var handler: InboundHandler?

    init(parent: Subscription, host: String, port: Int) {
        self.host = host
        self.port = port
        self.parent = parent
    }

    deinit {
        ROS_DEBUG("InboundConnection deinit")
        handler?.parent = nil
    }

    func initialize(owner: TransportPublisherLink) {
        self.link = owner
        let bootstrap = ClientBootstrap(group: threadGroup)
            // Enable SO_REUSEADDR.
            .channelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
            .channelInitializer { channel in
                channel.pipeline.add(handler: InboundHandler(parent: self))
        }

        do {
            self.channel = try bootstrap.connect(host: host, port: port).wait()
        } catch {
            ROS_ERROR("bootstrap failed: \(error)")
        }
    }

    func getTransportInfo() -> String {
        return "TCPROS connection on port \(channel?.localAddress?.port ?? 0) to [\(remoteAddress)]"
    }

    func writeHeader(keyVals: StringStringMap) -> EventLoopFuture<Void> {
        let buffer = Header.write(keyVals: keyVals)
        do {
            let sizeBuffer = try BinaryEncoder.encode(UInt32(buffer.count))
            return write(buffer: sizeBuffer + buffer)
        } catch {
            fatalError(error.localizedDescription)
        }
    }

    func write(buffer: [UInt8]) -> EventLoopFuture<Void> {
        guard let channel = channel else {
            ROS_ERROR("connection dropped")
            let promise: EventLoopPromise<Void> = threadGroup.next().newPromise()
            promise.fail(error: ConnectionError.connectionDropped)
            return promise.futureResult
        }
        var buf = channel.allocator.buffer(capacity: buffer.count)
        buf.write(bytes: buffer)
        return channel.writeAndFlush(buf)
    }

    var remoteAddress: String {
        let host = channel?.remoteAddress?.host ?? "unknown"
        let port = channel?.remoteAddress?.port ?? 0
        return "\(host):\(port)"
    }

    func drop(reason: DropReason) {
        if dropped.compareAndExchange(expected: false, desired: true) {
            ROS_DEBUG("Connection::drop - \(reason)")
            channel?.close().whenComplete {
                ROS_DEBUG("channel closed")
            }
            channel = nil
            link = nil
        }
    }

    func onHeaderReceived(header: Header) {
        ROS_DEBUG("onHeaderReceived")

        guard let link = link else {
            fatalError("onHeaderReceived has no link")
        }

        if !link.setHeader(header: header) {
            drop(reason: .headerError)
        }
    }

    final class InboundHandler: ChannelInboundHandler {
        public typealias InboundIn = ByteBuffer

        weak var parent: InboundConnection?

        init(parent: InboundConnection) {
            self.parent = parent
            parent.handler = self
        }

        func channelInactive(ctx: ChannelHandlerContext) {
            ROS_DEBUG("InboundHandler inactive")
            parent?.drop(reason: .transportDisconnect)
        }

        func channelRead(ctx: ChannelHandlerContext, data: NIOAny) {

            var buffer = self.unwrapInboundIn(data)
            guard let len: UInt32 = buffer.readInteger(endianness: .little) else {
                fatalError()
            }
            if len > buffer.readableBytes {
                ROS_ERROR("Received length \(buffer.readableBytes) < \(len) [\(self.parent?.remoteAddress ?? "uknown host")]")
                _ = ctx.close()
            }

            guard let p = parent, let link = parent?.link else {
                return
            }

            if link.header == nil {
                if let headerData = buffer.readBytes(length: Int(len)) {
                    let header = Header()
                    if !header.parse(buffer: headerData) {
                        p.drop(reason: .headerError)
                    } else {
                        if let error = header.getValue(key: "error") {
                            ROS_ERROR("Received error message in header for connection to [\(p.remoteAddress)]]: [\(error)]")
                            p.drop(reason: .headerError)
                        } else {
                            p.onHeaderReceived(header: header)
                        }
                    }
                }

            } else if let rawMessage = buffer.readBytes(length: Int(len)) {
                let m = SerializedMessage(buffer: rawMessage)
                if let sub = p.parent {
                    let drops = sub.handle(message: m,
                                           connectionHeader: link.header!.getValues(),
                                           link: link)
                }
            }
        }
    }
}

