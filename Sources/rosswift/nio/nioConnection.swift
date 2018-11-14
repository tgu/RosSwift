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
    case TransportDisconnect
    case HeaderError
    case Destructing
}

protocol ConnectionProtocol {

}


extension nio {

typealias ReadFinishedFunc = (Connection, [UInt8], Int, Bool) -> Void
typealias WriteFinishedFunc = (Connection) -> Void
typealias DropFunc = (Notification) -> Void




    final class Connection: ConnectionProtocol {


    static let dropNotification = Notification.Name(rawValue: "dropConnection")


    var channel : Channel
    var header_ : Header
    var sending_header_error = false

    init(transport: Channel, header: Header) {
        self.channel = transport
        self.header_ = header
    }

    var remoteAddress : String {
        let host = channel.remoteAddress?.host ?? "unknown"
        let port = channel.remoteAddress?.port ?? 0
        return "\(host):\(port)"
    }

    var callerID: String {
        if let callerid = header_.getValue(key: "callerid") {
            return callerid
        }

        return "unknown"
    }

    var remoteString: String {
        return "callerid=[\(callerID)] address=[\(remoteAddress)]"
    }

    func sendHeaderError(_ error_msg: String)
    {
        var m = M_string()
        m["error"] = error_msg

        writeHeader(key_vals: m).whenComplete {
            ROS_DEBUG("writeHeader finished")
        }
        sending_header_error = true
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
        var buf = channel.allocator.buffer(capacity: buffer.count)
        buf.write(bytes: buffer)
//        ROS_DEBUG("\(#file):\(#line)  write from \(self.channel.localAddress) => \(self.channel.remoteAddress)")
        return channel.writeAndFlush(buf)
    }

}


}
