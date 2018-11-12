//
//  TransportHints.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation

public class TransportHints {
    var transports_ = [String]()
    var options_ = [String:String]()

    public init() {
        
    }

    func reliable() -> TransportHints {
        let _ = tcp()
        return self
    }

    func tcp() -> TransportHints {
        transports_.append("TCP")
        return self
    }

    func tcpNoDelay(nodelay: Bool = true) -> TransportHints {
        options_["tcp_nodelay"] = nodelay ? "true" : "false"
        return self
    }

    func getTCPNoDelay() -> Bool {
        if let delay = options_["tcp_nodelay"] {
            return delay == "true"
        }
        return false
    }

    func maxDatagramSize() -> UInt32 {
        if let mds = options_["max_datagram_size"], let md = UInt32(mds) {
            return md
        }

        return 0
    }

    func unreliable() -> TransportHints {
        let _ = udp()
        return self
    }

    func udp() -> TransportHints {
        transports_.append("UDP")
        return self
    }

    func getTransports() -> [String] {
        return transports_
    }

    func getOptions() -> [String:String] {
        return options_
    }
}
