//
//  ServiceHandler.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-11-17.
//

import Foundation
import NIO
import StdMsgs

internal final class ServiceHandler: ChannelInboundHandler {
    enum ServiceState {
        case header
        case message
    }
    var state: ServiceState = .header

    typealias InboundIn = ByteBuffer

    func channelRead(context: ChannelHandlerContext, data: NIOAny) {
        var buffer = self.unwrapInboundIn(data)
        switch state {
        case .header:
            guard let len: UInt32 = buffer.readInteger(endianness: .little) else {
                fatalError("Received an invalid TCPROS header. Invalid count")
            }
            precondition(len <= buffer.readableBytes)

            var readMap = [String: String]()

            while buffer.readableBytes > 0 {
                guard let topicLen: UInt32 = buffer.readInteger(endianness: .little) else {
                    fatalError("Received an invalid TCPROS header. Invalid string")
                }

                guard let line = buffer.readString(length: Int(topicLen)) else {
                    fatalError("Received an invalid TCPROS header. Each line must have an equals sign.")
                }

                guard let equalIndex = line.firstIndex(of: "=") else {
                    fatalError("Received an invalid TCPROS header. Each line must have an equals sign.")
                }
                let key = String(line.prefix(upTo: equalIndex))
                let value = String(line.suffix(from: equalIndex).dropFirst())
                readMap[key] = value
            }
            ROS_DEBUG(readMap.debugDescription)
            state = .message
        case .message:
            guard let ok: UInt8 = buffer.readInteger(endianness: .little) else {
                fatalError("Received an invalid service message")
            }
            guard let len: UInt32 = buffer.readInteger(endianness: .little) else {
                fatalError("Received an invalid service message")
            }
            precondition(len <= buffer.readableBytes)
            if ok != 0 {
                ROS_DEBUG("missing logic for non-ok return")
            }
            if let rawMessage = buffer.readBytes(length: Int(len)) {
                let m = SerializedMessage(buffer: rawMessage)
                ROS_DEBUG("Servce response \(m)")
            }
        }
    }

}
