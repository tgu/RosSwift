//
//  ServiceServerLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-10.
//

import BinaryCoder
import Foundation
import NIO
import StdMsgs

final class ServiceServerLink: ChannelInboundHandler {
        var channel: Channel?
        let serviceName: String
        let persistent: Bool
        let requestMd5sum: String
        let responseMd5sum: String

        let extraOutgoingHeaderValues: StringStringMap?
        let headerWritten: Bool
        let headerRead: Bool

        private var isDropped: Bool

        init(serviceName: String,
             persistent: Bool,
             requestMd5sum: String,
             responseMd5sum: String,
             headerValues: StringStringMap?) {

            self.serviceName = serviceName
            self.persistent = persistent
            self.requestMd5sum = requestMd5sum
            self.responseMd5sum = responseMd5sum
            extraOutgoingHeaderValues = headerValues
            headerWritten = false
            headerRead = false
            isDropped = false
        }

        func initialize(channel: Channel) {
            self.channel = channel

            var header = StringStringMap()
            header["service"] = serviceName
            header["md5sum"] = requestMd5sum
            header["callerid"] = Ros.ThisNode.getName()
            header["persistent"] = persistent ? "1" : "0"
            if let extra = extraOutgoingHeaderValues {
                for item in extra {
                    header[item.key] = item.value
                }
            }

            let buffer = Header.write(keyVals: header)
            do {
                let sizeBuffer = try BinaryEncoder.encode(UInt32(buffer.count))
                var buf = channel.allocator.buffer(capacity: buffer.count + 4)
                buf.write(bytes: sizeBuffer + buffer)
                let data = IOData.byteBuffer(buf)

                channel.writeAndFlush(data).whenFailure { error in
                    ROS_DEBUG("ServiceServerLink, write failed to \(channel.remoteAddress)\nerror: \(error))")
                }
            } catch {
                ROS_ERROR("encode failed: \(error)")
            }

        }

    #if os(OSX)

        @objc
        func onConnectionDropped(note: NSNotification) {
            ROS_ERROR("\(onConnectionDropped) is not fully implemented")
            isDropped = true
            ServiceManager.instance.removeServiceServerLink(client: self)
        }

    #endif

        func isValid() -> Bool {
            return !isDropped
        }

        func call(req: SerializedMessage) -> EventLoopFuture<SerializedMessage> {

            let promise: EventLoopPromise<SerializedMessage> = channel!.eventLoop.newPromise()
            guard let c = channel else {
                promise.fail(error: ServiceError.invalidInput("ServiceServerLink::call has no connection"))
                return promise.futureResult
            }

            var buffer = c.allocator.buffer(capacity: req.buf.count)
            buffer.write(bytes: req.buf)

            c.writeAndFlush(buffer).whenFailure { error in
                ROS_DEBUG("ServiceServerLink \(#line), write failed to \(c.remoteAddress)\nerror: \(error))")
            }
            c.closeFuture.whenComplete {
                if self.response.buf.isEmpty {
                    promise.fail(error: ServiceError.noResponse)
                } else {
                    promise.succeed(result: self.response)
                }
            }

            return promise.futureResult
        }

        enum ServiceState {
            case header
            case message
        }
        var state: ServiceState = .header
        var response = SerializedMessage()

        typealias InboundIn = ByteBuffer

        func channelRead(ctx: ChannelHandlerContext, data: NIOAny) {
            var buffer = self.unwrapInboundIn(data)
            while buffer.readableBytes > 0 {
            switch state {
            case .header:
                guard let len: UInt32 = buffer.readInteger(endianness: .little) else {
                    fatalError("Received an invalid TCPROS header. Invalid count")
                }
                precondition(len <= buffer.readableBytes)

                var readMap = [String: String]()
                let leave = buffer.readableBytes - Int(len)
                while buffer.readableBytes > leave {
                    guard let topicLen: UInt32 = buffer.readInteger(endianness: .little) else {
                        fatalError("Received an invalid TCPROS header. Invalid string")
                    }

                    guard let line = buffer.readString(length: Int(topicLen)) else {
                        ROS_DEBUG("Received an invalid TCPROS header. Each line must have an equals sign.")
                        _ = ctx.close()
                        return
                    }

                    guard let equalIndex = line.index(of: "=") else {
                        fatalError("Received an invalid TCPROS header. Each line must have an equals sign.")
                    }
                    let key = String(line.prefix(upTo: equalIndex))
                    let value = String(line.suffix(from: equalIndex).dropFirst())
                    readMap[key] = value
                }
                state = .message
            case .message:
                guard let ok: UInt8 = buffer.readInteger(endianness: .little) else {
                    fatalError("Received malformed message")
                }
                ROS_DEBUG("OK = \(ok)")
                if let rawMessage = buffer.readBytes(length: buffer.readableBytes) {
                    response = SerializedMessage(buffer: rawMessage)
                }
                ctx.close()
            }
            }
        }

}
