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

protocol ConnectionProtocol {

}

extension Nio {

typealias ReadFinishedFunc = (Connection, [UInt8], Int, Bool) -> Void
typealias WriteFinishedFunc = (Connection) -> Void
typealias DropFunc = (Notification) -> Void

    final class Connection: ConnectionProtocol {

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
        if let callerid = header.getValue(key: "callerid") {
            return callerid
        }

        return "unknown"
    }

    var remoteString: String {
        return "callerid=[\(callerID)] address=[\(remoteAddress)]"
    }

    func getTransportInfo() -> String {
        return "TCPROS connection on port \(channel.localAddress?.port ?? 0) to [\(remoteAddress)]"
    }

    func sendHeaderError(_ msg: String) {
        var m = StringStringMap()
        m["error"] = msg

        writeHeader(keyVals: m).whenComplete {
            ROS_DEBUG("writeHeader finished")
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
        var buf = channel.allocator.buffer(capacity: buffer.count)
        buf.write(bytes: buffer)
        return channel.writeAndFlush(buf)
    }

}

}
