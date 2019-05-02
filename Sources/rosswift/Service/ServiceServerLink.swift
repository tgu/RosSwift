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

internal final class ServiceServerLink: ChannelInboundHandler {
        var channel: Channel?
        let serviceName: String
        let persistent: Bool
        let requestMd5sum: String
        let responseMd5sum: String

        let extraOutgoingHeaderValues: StringStringMap?
        let headerWritten: Bool
        let headerRead: Bool

        private var isDropped: Bool
    private let ros: Ros

    init(ros: Ros, serviceName: String,
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
        self.ros = ros
        }

        func initialize(channel: Channel) {
            self.channel = channel

            var header = StringStringMap()
            header["service"] = serviceName
            header["md5sum"] = requestMd5sum
            header["callerid"] = ros.name
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
                buf.writeBytes(sizeBuffer + buffer)
                let data = IOData.byteBuffer(buf)

                channel.writeAndFlush(data).whenFailure { error in
                    ROS_DEBUG("ServiceServerLink, write failed to \(String(describing: channel.remoteAddress))\nerror: \(error))")
                }
            } catch {
                ROS_ERROR("encode failed: \(error)")
            }

        }

    #if os(OSX)

        @objc
        func onConnectionDropped(note: NSNotification) {
            ROS_ERROR(#""onConnectionDropped" is not fully implemented"#)
            isDropped = true
            ros.serviceManager.removeServiceServerLink(client: self)
        }

    #endif

        func isValid() -> Bool {
            return !isDropped
        }

        func call(req: SerializedMessage) -> EventLoopFuture<SerializedMessage> {

            let promise: EventLoopPromise<SerializedMessage> = channel!.eventLoop.makePromise()
            guard let c = channel else {
                promise.fail(ServiceError.invalidInput("ServiceServerLink::call has no connection"))
                return promise.futureResult
            }

            var buffer = c.allocator.buffer(capacity: req.buf.count)
            buffer.writeBytes(req.buf)

            c.writeAndFlush(buffer).whenFailure { error in
                ROS_DEBUG("ServiceServerLink \(#line), write failed to \(String(describing: c.remoteAddress))\nerror: \(error))")
            }
            c.closeFuture.whenComplete { result in
                if self.response.buf.isEmpty {
                    promise.fail(ServiceError.noResponse)
                } else {
                    promise.succeed(self.response)
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

        func channelRead(context: ChannelHandlerContext, data: NIOAny) {
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
                        _ = context.close()
                        return
                    }

                    guard let equalIndex = line.firstIndex(of: "=") else {
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
                let _ = context.close()
            }
            }
        }

}
