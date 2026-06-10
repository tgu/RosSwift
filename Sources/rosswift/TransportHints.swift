//
//  TransportHints.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

/// Transport configuration for a subscription/publication. A value type:
/// it is configured at construction and read during connection negotiation,
/// so it needs neither reference semantics nor locking. The builder methods
/// return a modified copy for fluent chaining.
struct TransportHints: Sendable {
    var transports: [String] = []
    var options: [String: String] = [:]

    public init() {}

    func reliable() -> TransportHints {
        return tcp()
    }

    func tcp() -> TransportHints {
        var copy = self
        copy.transports.append("TCP")
        return copy
    }

    func tcpNoDelay(nodelay: Bool = true) -> TransportHints {
        var copy = self
        copy.options["tcp_nodelay"] = nodelay ? "true" : "false"
        return copy
    }

    func getTCPNoDelay() -> Bool {
        return options["tcp_nodelay"] == "true"
    }

    func maxDatagramSize() -> UInt32 {
        if let mds = options["max_datagram_size"], let mdSize = UInt32(mds) {
            return mdSize
        }
        return 0
    }

    func unreliable() -> TransportHints {
        return udp()
    }

    func udp() -> TransportHints {
        var copy = self
        copy.transports.append("UDP")
        return copy
    }

    func getTransports() -> [String] {
        return transports
    }

    func getOptions() -> [String: String] {
        return options
    }
}
