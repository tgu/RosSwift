//
//  SubscriptionCallbackHelper.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import BinaryCoder
import StdMsgs
import RosTime

protocol SubscriptionCallbackHelper {
    var id: ObjectIdentifier { get }
    func deserialize(data: [UInt8]) -> Message?
    func call(msg: Message, item: SubscriptionQueue.Item)
}

final class SubscriptionCallbackHelperT<M: Message>: SubscriptionCallbackHelper {

    typealias Callback = (M) -> Void

    let callback: Callback

    var id: ObjectIdentifier {
        return ObjectIdentifier(self)
    }

    init(callback: @escaping Callback ) {
        self.callback = callback

    }

    func deserialize(data: [UInt8]) -> Message? {
        return try? BinaryDecoder.decode(M.self, data: data)
    }

    func call(msg: Message, item: SubscriptionQueue.Item) {
        if let message = msg as? M {
            callback(message)
        }
    }

}


final class SubscriptionEventCallbackHelperT<M: Message>: SubscriptionCallbackHelper {

    typealias Callback = (MessageEvent<M>) -> Void

    let callback: Callback

    var id: ObjectIdentifier {
        return ObjectIdentifier(self)
    }

    init(callback: @escaping Callback ) {
        self.callback = callback
    }

    func deserialize(data: [UInt8]) -> Message? {
        return try? BinaryDecoder.decode(M.self, data: data)
    }

    func call(msg: Message, item: SubscriptionQueue.Item) {
        let time = item.receiptTime
        if let message = msg as? M {
            let event = MessageEvent(message: message, header: item.deserializer.connectionHeader, receiptTime: time)
            callback(event)
        } else {
            ROS_ERROR("Trying to send message of type \(type(of: msg)) to a subscriber that wants \(type(of: M.self))")
        }
    }

}


