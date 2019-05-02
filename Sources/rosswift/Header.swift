//
//  Header.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import BinaryCoder
import Foundation
import StdMsgs

public final class Header {
    public var headers = StringStringMap()

    public func getValues() -> StringStringMap {
        return headers
    }

    public func getValue(key: String) -> String? {
        return headers[key]
    }

    public func parse(buffer: [UInt8]) -> Bool {
        var indx = 0
        while indx < buffer.count {
            let len = UInt32(buffer[indx]) |
                UInt32(buffer[indx + 1]) << 8 |
                UInt32(buffer[indx + 2]) << 16 |
                UInt32(buffer[indx + 3]) << 24
            indx += 4
            if len > 1_000_000 {
                ROS_DEBUG("Received an invalid TCPROS header.  Each element must be prepended by a 4-byte length.")
                return false
            }

            let buf = buffer[indx..<indx + Int(len)]

            guard let line = String(bytes: buf, encoding: .utf8) else {
                ROS_DEBUG("Received an invalid TCPROS header.  invalid string")
                return false
            }
            indx += Int(len)
            guard let eqs = line.firstIndex(of: "=") else {
                ROS_DEBUG("Received an invalid TCPROS header.  Each line must have an equals sign.")
                return false
            }
            let key = String(line.prefix(upTo: eqs))
            let value = String(line.suffix(from: eqs).dropFirst())
            headers[key] = value
        }

        return true
    }

    static func write(keyVals: StringStringMap) -> [UInt8] {
        var data = [UInt8]()
        for (key, val) in keyVals {
            let str = "\(key)=\(val)"
            do {
                data.append(contentsOf: try BinaryEncoder.encode(str))
            } catch {
                ROS_ERROR("encoding \(str) failed with error: \(error)")
            }
        }
        return data
    }
}
