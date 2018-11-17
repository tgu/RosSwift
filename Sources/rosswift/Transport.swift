//
//  Transport.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation

typealias Callback = (Transport) -> Void

protocol Transport {

    func enableRead()
    func disableRead()
    func setDisconnectCallback(cb: @escaping Callback)
    func setReadCallback(cb: @escaping Callback)
    func setWriteCallback(cb: @escaping Callback)
    func close()
    func getServerPort() -> Int32
    func getTransportInfo() -> String
    func read(into buffer: inout [UInt8], toRead: Int) -> Int
    func parseHeader(header: Header)
    func requiresHeader() -> Bool
    func enableWrite()
    func disableWrite()
    func write(buffer: [UInt8], size: Int) -> Int
    func getType() -> String
}

extension Transport {
    func writeSlice(buffer: ArraySlice<UInt8>, size: Int) -> Int {
        return write(buffer: [UInt8](buffer), size: size)
    }
}
