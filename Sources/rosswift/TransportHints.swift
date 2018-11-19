//
//  TransportHints.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation

public final class TransportHints {
    var transports = [String]()
    var options = [String: String]()

    public init() {

    }

    func reliable() -> TransportHints {
        _ = tcp()
        return self
    }

    func tcp() -> TransportHints {
        transports.append("TCP")
        return self
    }

    func tcpNoDelay(nodelay: Bool = true) -> TransportHints {
        options["tcp_nodelay"] = nodelay ? "true" : "false"
        return self
    }

    func getTCPNoDelay() -> Bool {
        if let delay = options["tcp_nodelay"] {
            return delay == "true"
        }
        return false
    }

    func maxDatagramSize() -> UInt32 {
        if let mds = options["max_datagram_size"], let mdSize = UInt32(mds) {
            return mdSize
        }

        return 0
    }

    func unreliable() -> TransportHints {
        _ = udp()
        return self
    }

    func udp() -> TransportHints {
        transports.append("UDP")
        return self
    }

    func getTransports() -> [String] {
        return transports
    }

    func getOptions() -> [String: String] {
        return options
    }
}
