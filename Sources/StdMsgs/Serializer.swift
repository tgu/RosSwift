//
//  Serializer.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-09.
//

import Foundation
import BinaryCoder

func serialize<T : BinaryCodable>(_ value: T) -> [UInt8] {
    let data = try! BinaryEncoder.encode(value)
    let buf = try! BinaryEncoder.encode(UInt32(data.count))
    return buf+data
}

func deserialize<T : BinaryCodable>(_ buffer: [UInt8]) -> T {
    let b = [UInt8](buffer.dropFirst(4))
    return try! BinaryDecoder.decode(T.self, data: b)
}
