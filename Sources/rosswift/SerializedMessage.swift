//
//  SerializedMessage.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-05.
//

import StdMsgs
import BinaryCoder
import Synchronization

public final class SerializedMessage: Sendable {

    private struct State: Sendable {
        var buf: [UInt8]
        var message: Message?
    }
    private let state: Mutex<State>

    public var buf: [UInt8] {
        state.withLock { $0.buf }
    }

    public var message: Message? {
        state.withLock { $0.message }
    }

    public var byteCount: Int {
        state.withLock { $0.buf.count }
    }

    public init() {
        state = Mutex(State(buf: [], message: nil))
    }

    public init(msg: Message) {
        let encoded: [UInt8]
        do {
            let code = try BinaryEncoder.encode(msg)
            encoded = try BinaryEncoder.encode(UInt32(code.count)) + code
        } catch {
            ROS_ERROR("failed to serialize \(msg)")
            encoded = []
        }
        state = Mutex(State(buf: encoded, message: msg))
    }

    public init(buffer: consuming [UInt8]) {
        state = Mutex(State(buf: buffer, message: nil))
    }

    /// Records the result of a successful deserialization so later callers
    /// (e.g. additional `MessageDeserializer`s sharing this `SerializedMessage`)
    /// can skip decoding. Releases the encoded bytes since they're no longer
    /// needed.
    internal func cacheDeserialized(_ message: Message) {
        state.withLock {
            $0.message = message
            $0.buf.removeAll()
        }
    }
}
