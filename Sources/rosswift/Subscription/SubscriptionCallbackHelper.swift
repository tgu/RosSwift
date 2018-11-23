//
//  SubscriptionCallbackHelper.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import BinaryCoder
import Foundation
import StdMsgs
import RosTime

public struct SubscriptionCallbackHelperCallParams<M: Message> {
    var event: MessageEvent<M>
}

public protocol SubscriptionCallbackHelper {
    var id: ObjectIdentifier { get }
    func deserialize(data: [UInt8]) -> Message?
    func call(msg: Message)
    func getTypeInfo() -> String
    func isConst() -> Bool
    func hasHeader() -> Bool
}

public final class SubscriptionCallbackHelperT<M: Message>: SubscriptionCallbackHelper {

    typealias Callback = (M) -> Void

    var callback: Callback

    public var id: ObjectIdentifier {
        return ObjectIdentifier(self)
    }

    init(callback: @escaping Callback ) {
        self.callback = callback

    }

    public func deserialize(data: [UInt8]) -> Message? {
        return try? BinaryDecoder.decode(M.self, data: data)
    }

    public func call(msg: Message) {
        if let message = msg as? M {
            callback(message)
        }
    }

    public func getTypeInfo() -> String {
        return String(String(reflecting: M.self).prefix(while: { $0 != "(" }))
    }

    public func isConst() -> Bool {
        return  false
    }

    public func hasHeader() -> Bool {
        return M.hasHeader
    }

}


public final class SubscriptionEventCallbackHelperT<M: Message>: SubscriptionCallbackHelper {

    typealias Callback = (MessageEvent<M>) -> Void

    var callback: Callback

    public var id: ObjectIdentifier {
        return ObjectIdentifier(self)
    }

    init(callback: @escaping Callback ) {
        self.callback = callback

    }

    public func deserialize(data: [UInt8]) -> Message? {
        return try? BinaryDecoder.decode(M.self, data: data)
    }

    public func call(msg: Message) {
        if let message = msg as? M {
            let event = MessageEvent(message: message, header: ["callerid":"unknown"], receiptTime: RosTime.Time())
            callback(event)
        }
    }

    public func getTypeInfo() -> String {
        return String(String(reflecting: M.self).prefix(while: { $0 != "(" }))
    }

    public func isConst() -> Bool {
        return  false
    }

    public func hasHeader() -> Bool {
        return M.hasHeader
    }

}


