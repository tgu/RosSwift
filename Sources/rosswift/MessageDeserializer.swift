//
//  MessageDeserializer.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-16.
//

import BinaryCoder
import Foundation
import StdMsgs

actor MessageDeserializer: Sendable {
    let helper: SubscriptionCallbackHelper
    let serializedMessage: SerializedMessage
    let connectionHeader: StringStringMap
    var message: Message? = nil
    
    init(helper: SubscriptionCallbackHelper, m: SerializedMessage, header: StringStringMap) {
        self.helper = helper
        self.serializedMessage = m
        self.connectionHeader = header
    }
    
    func deserialize() -> Message? {
        if message != nil {
            return message
        }
        
        if let msg = serializedMessage.message {
            message = msg
            return message
        }
        
        if serializedMessage.buf.isEmpty && serializedMessage.byteCount > 0 {
            // If the buffer has been reset it means we tried to deserialize and failed
            return message
        }
        
        message = helper.deserialize(data: serializedMessage.buf)
        if let cached = message {
            serializedMessage.cacheDeserialized(cached)
        }

        return message
    }
    
}

func deserializeMessage<M: Message>(m: SerializedMessage) throws -> M {
    let b = [UInt8](m.buf.dropFirst(4))
    return try BinaryDecoder.decode(M.self, data: b)
}
