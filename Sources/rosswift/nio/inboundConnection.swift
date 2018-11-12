//
//  inboundConnection.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-23.
//

import Foundation
import StdMsgs
import NIO
import NIOConcurrencyHelpers
import BinaryCoder

enum ConnectionError: Error {
    case connectionDropped
}

class InboundConnection: ConnectionProtocol {

//    var bootstrap : ClientBootstrap?
    var channel: Channel?
    var dropped = Atomic<Bool>(value: false)
    weak var parent : Subscription?
    weak var link: TransportPublisherLink?
    let host: String
    let port: Int
    var handler : InboundHandler?

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
        let bootstrap = ClientBootstrap(group: thread_group)
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

    func writeHeader(key_vals: M_string) -> EventLoopFuture<Void> {
        let buffer = Header.write(key_vals: key_vals)
        do {
            let sb = try BinaryEncoder.encode(UInt32(buffer.count))
            return write(buffer: sb+buffer)
        } catch {
            fatalError(error.localizedDescription)
        }
    }

    func write(buffer: [UInt8]) -> EventLoopFuture<Void> {
        guard let channel = channel else {
            ROS_ERROR("connection dropped")
            let promise : EventLoopPromise<Void> = thread_group.next().newPromise()
            promise.fail(error: ConnectionError.connectionDropped)
            return promise.futureResult
        }
        var buf = channel.allocator.buffer(capacity: buffer.count)
        buf.write(bytes: buffer)
//        ROS_DEBUG("\(#file):\(#line)  write from \(channel.localAddress) => \(channel.remoteAddress)")
        return channel.writeAndFlush(buf)
    }

    var remoteAddress : String {
        let host = channel?.remoteAddress?.host ?? "unknown"
        let port = channel?.remoteAddress?.port ?? 0
        return "\(host):\(port)"
    }


    func drop(reason: DropReason) {
        if dropped.compareAndExchange(expected: false, desired: true) {
            ROS_DEBUG("Connection::drop - \(reason)")
            channel?.close()
            channel = nil
            link = nil
        }
    }

    func onHeaderReceived(header: Header) {
        ROS_DEBUG("onHeaderReceived")

        guard let link = link else {
            fatalError()
        }

        if !link.setHeader(header: header) {
            drop(reason: .HeaderError)
        }
    }

    class InboundHandler: ChannelInboundHandler {
        public typealias InboundIn = ByteBuffer

        weak var parent : InboundConnection?

        init(parent: InboundConnection) {
            self.parent = parent
            parent.handler = self
        }

        func channelInactive(ctx: ChannelHandlerContext) {
            ROS_DEBUG("InboundHandler inactive")
            parent?.drop(reason: .TransportDisconnect)
        }

        func channelRead(ctx: ChannelHandlerContext, data: NIOAny) {

            ROS_DEBUG("messge from \(ctx.channel.remoteAddress!) => \(ctx.channel.localAddress!)")
            var buffer = self.unwrapInboundIn(data)
            guard let len : UInt32 = buffer.readInteger(endianness: .little) else {
                fatalError()
            }
            if len > buffer.readableBytes {
                ROS_ERROR("Received length \(buffer.readableBytes) < \(len) [\(self.parent?.remoteAddress)]")
                ctx.close()
            }

            guard let p = parent, let link = parent?.link else {
                return
            }

            if link.header == nil {
                if let headerData = buffer.readBytes(length: Int(len)) {
                    let header = Header()
                    if !header.parse(buffer: headerData) {
                        p.drop(reason: .HeaderError)
                    } else {
                        if let error_val = header.getValue(key: "error") {
                            ROS_ERROR("Received error message in header for connection to [\(p.remoteAddress)]]: [\(error_val)]")
                            p.drop(reason: .HeaderError);
                        } else {
                            p.onHeaderReceived(header: header)
                        }
                    }
                }
                
            } else {


                if let rawMessage = buffer.readBytes(length: Int(len)) {
                    let m = SerializedMessage(buffer: rawMessage)
                    if let sub = p.parent {
                        let drops = sub.handle(message: m, ser: true, nocopy: false, connection_header: link.header!.getValues(), link: link)
                    }
                }
            }

        }
    }

}
