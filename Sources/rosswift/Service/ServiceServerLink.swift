//
//  ServiceServerLink.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-10.
//

import Foundation
import StdMsgs
import NIO
import BinaryCoder
 



class ServiceServerLink: ChannelInboundHandler {
        var channel : Channel?
        var service_name_ : String
        var persistent_ : Bool
        var request_md5sum_ : String
        var response_md5sum_ : String

        var extra_outgoing_header_values_ : M_string?
        var header_written_ : Bool
        var header_read_ : Bool

        var dropped_ : Bool


        init(service_name: String, persistent: Bool, request_md5sum: String, response_md5sum: String, header_values: M_string?) {
            service_name_ = service_name
            persistent_ = persistent
            request_md5sum_ = request_md5sum
            response_md5sum_ = response_md5sum
            extra_outgoing_header_values_ = header_values
            header_written_ = false
            header_read_ = false
            dropped_ = false
        }


        func initialize(channel: Channel) {
//            ROS_DEBUG("initialize")
            self.channel = channel

            var header = M_string()
            header["service"] = service_name_
            header["md5sum"] = request_md5sum_
            header["callerid"] = Ros.this_node.getName()
            header["persistent"] = persistent_ ? "1" : "0"
            if let extra = extra_outgoing_header_values_ {
                for item in extra {
                    header[item.key] = item.value
                }
            }

            let buffer = Header.write(key_vals: header)
            do {
                let sb = try BinaryEncoder.encode(UInt32(buffer.count))
                var buf = channel.allocator.buffer(capacity: buffer.count+4)
                buf.write(bytes: sb+buffer)
                let data = IOData.byteBuffer(buf)

                channel.writeAndFlush(data).whenFailure { (error) in
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
//            guard let conn = connection_ else {
//                return
//            }
//            ROS_DEBUG("Service client from [\(conn.getRemoteString())] for [\(service_name_)] dropped")

            dropped_ = true;
//            clearCalls()

            ServiceManager.instance.removeServiceServerLink(client: self)
        }

    #endif

        func isValid() -> Bool {
            return !dropped_;
        }



        func call(req: StdMsgs.SerializedMessage) -> EventLoopFuture<SerializedMessage> {

            let promise : EventLoopPromise<SerializedMessage> = channel!.eventLoop.newPromise()
            guard let c = channel else {
                promise.fail(error: ServiceError.invalidInput("ServiceServerLink::call has no connection"))
                return promise.futureResult
            }

            var buffer = c.allocator.buffer(capacity: req.buf.count)
            buffer.write(bytes: req.buf)
//            ROS_DEBUG("ServiceCall to [\(c.remoteAddress!.host):\(c.remoteAddress!.port!)]: \(req.message!)")

            c.writeAndFlush(buffer).whenFailure { (error) in
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
        var state : ServiceState = .header
        var response = SerializedMessage()

        typealias InboundIn = ByteBuffer

        func channelRead(ctx: ChannelHandlerContext, data: NIOAny) {
            var buffer = self.unwrapInboundIn(data)
            while buffer.readableBytes > 0 {
            switch state {
            case .header:
//                ROS_DEBUG("readable bytes = \(buffer.readableBytes)")
//                ROS_DEBUG(buffer.getBytes(at: buffer.readerIndex, length: buffer.readableBytes))
                guard let len : UInt32 = buffer.readInteger(endianness: .little) else {
                    fatalError()
                }
                precondition(len <= buffer.readableBytes)

                var readMap = [String:String]()

                var remaining = len

                let leave = buffer.readableBytes - Int(len)

                while buffer.readableBytes > leave {
                    guard let topicLen : UInt32 = buffer.readInteger(endianness: .little) else {
                        ROS_DEBUG("Received an invalid TCPROS header.  invalid string")
                        fatalError()
                    }

                    guard let line = buffer.readString(length: Int(topicLen)) else {
                        ROS_DEBUG("Received an invalid TCPROS header.  Each line must have an equals sign.")
                        ctx.close()
                        return
                    }

                    guard let eq = line.index(of: "=") else {
                        ROS_DEBUG("Received an invalid TCPROS header.  Each line must have an equals sign.")
                        fatalError()
                    }
                    let key = String(line.prefix(upTo: eq))
                    let value = String(line.suffix(from: eq).dropFirst())
                    readMap[key] = value
                }
//                ROS_DEBUG(readMap)
                state = .message
            case .message:
//                ROS_DEBUG("readable bytes = \(buffer.readableBytes)")
//                ROS_DEBUG(buffer.getBytes(at: buffer.readerIndex, length: buffer.readableBytes))
                guard let ok : UInt8 = buffer.readInteger(endianness: .little) else {
                    fatalError()
                }
                ROS_DEBUG("OK = \(ok)")
//                guard let len : UInt32 = buffer.readInteger(endianness: .little) else {
//                    fatalError()
//                }
//                precondition(len <= buffer.readableBytes)
                if let rawMessage = buffer.readBytes(length: buffer.readableBytes) {
                    response = SerializedMessage(buffer: rawMessage)
//                    ROS_DEBUG("Received \(response.buf), from \(ctx.remoteAddress)")
                }
                ctx.close()
            }
            }
        }


}


