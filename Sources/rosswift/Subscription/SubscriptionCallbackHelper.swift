//
//  SubscriptionCallbackHelper.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import BinaryCoder
import StdMsgs
import RosTime

internal protocol SubscriptionCallbackHelper {
    var id: ObjectIdentifier { get }
    func deserialize(data: [UInt8]) -> Message?
    func call(msg: Message, item: SubscriptionQueue.Item)
    var hasHeader: Bool { get }
}

internal final class SubscriptionCallbackHelperT<M: Message>: SubscriptionCallbackHelper {

    typealias Callback = (M) -> Void

    let callback: Callback

    public var id: ObjectIdentifier {
        return ObjectIdentifier(self)
    }

    init(callback: @escaping Callback ) {
        self.callback = callback

    }

    public func deserialize(data: [UInt8]) -> Message? {
        return try? BinaryDecoder.decode(M.self, data: data)
    }

    public func call(msg: Message, item: SubscriptionQueue.Item) {
        if let message = msg as? M {
            callback(message)
        }
    }

    public let hasHeader = M.hasHeader

}


internal final class SubscriptionEventCallbackHelperT<M: Message>: SubscriptionCallbackHelper {

    typealias Callback = (MessageEvent<M>) -> Void

    let callback: Callback

    public var id: ObjectIdentifier {
        return ObjectIdentifier(self)
    }

    init(callback: @escaping Callback ) {
        self.callback = callback
    }

    public func deserialize(data: [UInt8]) -> Message? {
        return try? BinaryDecoder.decode(M.self, data: data)
    }

    public func call(msg: Message, item: SubscriptionQueue.Item) {
        let time = item.receiptTime
        if let message = msg as? M {
            let event = MessageEvent(message: message, header: item.deserializer.connectionHeader, receiptTime: time)
            callback(event)
        } else {
            ROS_ERROR("Trying to send message of type \(type(of: msg)) to a subscriber that wants \(type(of: M.self))")
        }
    }

    public let hasHeader = M.hasHeader

}


