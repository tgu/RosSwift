//
//  SerializedMessage.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-05.
//

import Foundation
import BinaryCoder
import LoggerAPI

public typealias TypeInfo = String

public final class SerializedMessage {
    public var buf : [UInt8]
    public var message : Message?
    public var typeInfo : TypeInfo? = nil

    public var num_bytes: Int {
        return buf.count
    }

    public init() {
        buf = [UInt8]()
        message = nil
    }

    public init(msg: Message) {
        self.message = msg
        self.typeInfo = String(String(reflecting: msg).prefix(while: { $0 != "(" }))

        do {
            let code = try BinaryEncoder.encode(msg)
            buf = try BinaryEncoder.encode(UInt32(code.count))
            buf.append(contentsOf: code)
        }
        catch {
            Log.error("failed to serialize \(msg)")
            self.buf = [UInt8]()
        }
    }

    public init(msg: Message, buffer: [UInt8]) {
        self.message = msg
        self.buf = buffer
    }

    public init(buffer: [UInt8]) {
        self.buf = buffer
        message = nil
    }
}

public typealias SerializationFunction = () -> SerializedMessage
