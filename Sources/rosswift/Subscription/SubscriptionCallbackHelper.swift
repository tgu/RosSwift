//
//  SubscriptionCallbackHelper.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import BinaryCoder
import StdMsgs
import RosTime
import Foundation

protocol SubscriptionCallbackHelper: Sendable {
    var id: UUID { get }
    func deserialize(data: [UInt8]) -> Message?
    func call(msg: Message, item: SubscriptionQueue.Item) async throws
}

struct CallBackError: Error, CustomStringConvertible {
    let expectedMsg: String
    let providedMsg: String
    
    var description: String {
        "Trying to send message of type \(providedMsg) to a subscriber that wants \(expectedMsg)"
    }
}

struct SubscriptionCallbackHelperT<M: Message>: SubscriptionCallbackHelper {
    
    typealias Callback = @Sendable (M) -> Void
    
    let callback: Callback
    let id = UUID()
    
    init(callback: @escaping Callback ) {
        self.callback = callback
        
    }
    
    func deserialize(data: [UInt8]) -> Message? {
        return try? BinaryDecoder.decode(M.self, data: data)
    }
    
    func call(msg: Message, item: SubscriptionQueue.Item) throws {
        if let message = msg as? M {
            callback(message)
        } else {
            throw CallBackError(expectedMsg: "\(type(of: msg))", providedMsg: "\(type(of: M.self))")
        }
        
    }
    
}


struct SubscriptionEventCallbackHelperT<M: Message>: SubscriptionCallbackHelper {
    
    typealias Callback = @Sendable (MessageEvent<M>) -> Void
    
    let callback: Callback
    let id = UUID()
    
    init(callback: @escaping Callback ) {
        self.callback = callback
    }
    
    func deserialize(data: [UInt8]) -> Message? {
        return try? BinaryDecoder.decode(M.self, data: data)
    }
    
    func call(msg: Message, item: SubscriptionQueue.Item) throws {
        let time = item.receiptTime
        if let message = msg as? M {
            let event = MessageEvent(message: message, header: item.deserializer.connectionHeader, receiptTime: time)
            callback(event)
        } else {
            throw CallBackError(expectedMsg: "\(type(of: msg))", providedMsg: "\(type(of: M.self))")
            //  ROS_ERROR("Trying to send message of type \(type(of: msg)) to a subscriber that wants \(type(of: M.self))")
        }
    }
    
}


