//
//  Connection.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation
import StdMsgs
import BinaryCoder
import NIO

enum DropReason {
    case transportDisconnect
    case headerError
    case destructing
}

final class Connection {

    var channel: Channel
    var header: Header
    var isSendingHeaderError = false

    init(transport: Channel, header: Header) {
        self.channel = transport
        self.header = header
    }

    var remoteAddress: String {
        let host = channel.remoteAddress?.host ?? "unknown"
        let port = channel.remoteAddress?.port ?? 0
        return "\(host):\(port)"
    }

    var callerID: String {
        if let callerid = header["callerid"] {
            return callerid
        }

        return "unknown"
    }

    var remoteString: String {
        return "callerid=[\(callerID)] address=[\(remoteAddress)]"
    }

    var transportInfo: String {
        return "TCPROS connection on port \(channel.localAddress?.port ?? 0) to [\(remoteAddress)]"
    }

    func sendHeaderError(_ msg: String) {
        writeHeader(keyVals: ["error": msg]).whenComplete { result in
            switch result {
            case .success:
                ROS_DEBUG("writeHeader finished")
            case .failure(let error):
                ROS_ERROR("Failure in sendHeaderError: \(error)")
            }
        }
        isSendingHeaderError = true
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
        let buf = channel.allocator.buffer(bytes: buffer)
        return channel.writeAndFlush(buf)
    }

}

