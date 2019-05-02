//
//  SerializedMessage.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-05.
//

import Foundation
import BinaryCoder
import LoggerAPI

public final class SerializedMessage {
    public var buf : [UInt8]
    public var message : Message?

    public var byteCount: Int {
        return buf.count
    }

    public init() {
        buf = []
        message = nil
    }
    
    public init(msg: Message) {
        self.message = msg

        do {
            let code = try BinaryEncoder.encode(msg)
            buf = try BinaryEncoder.encode(UInt32(code.count))
            buf.append(contentsOf: code)
        }
        catch {
            Log.error("failed to serialize \(msg)")
            self.buf = []
        }
    }

    public init(buffer: [UInt8]) {
        self.buf = buffer
        message = nil
    }
}
