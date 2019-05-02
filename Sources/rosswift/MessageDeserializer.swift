//
//  MessageDeserializer.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-16.
//

import BinaryCoder
import Foundation
import StdMsgs

final class MessageDeserializer {
    let helper: SubscriptionCallbackHelper
    let serializedMessage: SerializedMessage
    let connectionHeader: StringStringMap
    let deserializeQueue = DispatchQueue(label: "deserializeQueue")
    var message: Message?

    init(helper: SubscriptionCallbackHelper, m: SerializedMessage, header: StringStringMap) {
        self.helper = helper
        self.serializedMessage = m
        self.connectionHeader = header
    }

    func deserialize() -> Message? {
        deserializeQueue.sync {
            if message != nil {
                return
            }

            if let msg = serializedMessage.message {
                message = msg
                return
            }

            if serializedMessage.buf.isEmpty && serializedMessage.byteCount > 0 {
                // If the buffer has been reset it means we tried to deserialize and failed
                return
            }

            message = helper.deserialize(data: serializedMessage.buf)
            serializedMessage.buf.removeAll()
            serializedMessage.message = message
        }

        return message
    }

}

func deserializeMessage<M: Message>(m: SerializedMessage)  throws -> M {
    let b = [UInt8](m.buf.dropFirst(4))
    return try BinaryDecoder.decode(M.self, data: b)
}
