//
//  MessageDeserializer.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-16.
//

import Foundation
import StdMsgs
import BinaryCoder

final class MessageDeserializer {
    var helper_ : SubscriptionCallbackHelper
    var serialized_message_ : SerializedMessage
    var connection_header_ : M_string
    let mutex_ = DispatchQueue(label: "mutex_")
    var msg_ : Message?

    init(helper: SubscriptionCallbackHelper, m: SerializedMessage, header: M_string) {
        self.helper_ = helper
        self.serialized_message_ = m
        self.connection_header_ = header
    }

    func deserialize() -> Message? {
        mutex_.sync {
            if msg_ != nil {
                return
            }

            if let msg = serialized_message_.message {
                msg_ = msg
                return
            }

            if serialized_message_.buf.count == 0 && serialized_message_.num_bytes > 0 {
                // If the buffer has been reset it means we tried to deserialize and failed
                return
            }

//            let params = SubscriptionCallbackHelperDeserializeParams(buffer: serialized_message_.buf, length: UInt32(serialized_message_.buf.count), connection_header: connection_header_)

            msg_ = helper_.deserialize(data: serialized_message_.buf)
            serialized_message_.buf.removeAll()
        }

        return msg_
    }

}


func deserializeMessage<M: Message>(m: SerializedMessage)  throws -> M {
    let b = [UInt8](m.buf.dropFirst(4))
    return try BinaryDecoder.decode(M.self, data: b)
}
