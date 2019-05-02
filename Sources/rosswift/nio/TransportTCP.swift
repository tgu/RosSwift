//
//  Master.swift
//  swiftros
//
//  Created by Thomas Gustafsson on 2018-03-19.
//

import Foundation
import NIO

import BinaryCoder

final class MessageDelimiterCodec: ByteToMessageDecoder {
    typealias InboundIn = ByteBuffer
    typealias InboundOut = ByteBuffer

    public var cumulationBuffer: ByteBuffer?

    func decode(context: ChannelHandlerContext, buffer: inout ByteBuffer) throws -> DecodingState {
        if let count: UInt32 = buffer.getInteger(at: buffer.readerIndex, endianness: .little) {
            if buffer.readableBytes >= count + 4 {
                context.fireChannelRead(self.wrapInboundOut(buffer.readSlice(length: Int(count + 4))!))
                return .continue
            }
        }
        ROS_DEBUG("MessageDelimiterCodec from \(String(describing: context.channel.remoteAddress)) => \(String(describing: context.channel.localAddress)) need more data")
        return .needMoreData
    }

    func decodeLast(context: ChannelHandlerContext, buffer: inout ByteBuffer, seenEOF: Bool) throws -> DecodingState {
        self.cumulationBuffer = nil
        return .continue
    }

}

final class HeaderMessageCodec: ByteToMessageDecoder {
    public typealias InboundIn = ByteBuffer
    public typealias InboundOut = StringStringMap

    public var cumulationBuffer: ByteBuffer?

    func decode(context: ChannelHandlerContext, buffer: inout ByteBuffer) throws -> DecodingState {
        guard let len: UInt32 = buffer.readInteger(endianness: .little) else {
            fatalError()
        }
        precondition(len <= buffer.readableBytes)

        var readMap = [String: String]()

        while buffer.readableBytes > 0 {
            guard let topicLen: UInt32 = buffer.readInteger(endianness: .little) else {
                ROS_DEBUG("Received an invalid TCPROS header.  invalid string")
                return .continue
            }

            guard let line = buffer.readString(length: Int(topicLen)) else {
                ROS_DEBUG("Received an invalid TCPROS header.  Each line must have an equals sign.")
                return .continue
            }

            guard let eq = line.firstIndex(of: "=") else {
                ROS_DEBUG("Received an invalid TCPROS header.  Each line must have an equals sign.")
                return .continue
            }
            let key = String(line.prefix(upTo: eq))
            let value = String(line.suffix(from: eq).dropFirst())
            readMap[key] = value
        }

        context.fireChannelRead(self.wrapInboundOut(readMap))
        return .continue
    }

    func decodeLast(context: ChannelHandlerContext, buffer: inout ByteBuffer, seenEOF: Bool) throws -> DecodingState {
        self.cumulationBuffer = nil
        return .continue
    }

}

public final class TransportTCP {

    final class Handler: ChannelInboundHandler {
        public typealias InboundIn = StringStringMap
        public typealias OutboundOut = ByteBuffer

        let callback: (StringStringMap) -> Void

        init(callback: @escaping (StringStringMap) -> Void) {
            self.callback = callback
        }

        public func channelRead(context: ChannelHandlerContext, data: NIOAny) {
            let buffer = self.unwrapInboundIn(data)
            callback(buffer)
        }
    }

    let bootstrap: ClientBootstrap

    public init() {
        self.bootstrap = ClientBootstrap(group: threadGroup.next())
            .channelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
    }

    public func connect(host: String, port: Int) -> EventLoopFuture<Channel> {
        return bootstrap.connect(host: host, port: port)
    }

}
