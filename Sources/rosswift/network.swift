//
//  network.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation
import NIO

extension Ros {

    struct network {
        static var g_host = ""
        static var g_tcpros_server_port: UInt16 = 0

        static func getHost() -> String {
            return g_host
        }

        static func initialize(remappings: M_string) {
            if let it = remappings["__hostname"] {
                g_host = it
            } else if let it = remappings["__ip"] {
                g_host = it
            }
            if let it = remappings["__tcpros_server_port"] {
                guard let tcpros_server_port = UInt16(it) else {
                    fatalError("__tcpros_server_port [\(it)] was not specified as a number within the 0-65535 range")
                }
                g_tcpros_server_port = tcpros_server_port
            }

            if g_host.isEmpty {
                g_host = determineHost()
            }
        }

        static func splitURI(uri: String, host: inout String, port: inout UInt16) -> Bool {
            var uri = uri  
            if uri.hasPrefix("http://") {
                uri = String(uri.dropFirst(7))
            } else if uri.hasPrefix("rosrpc://") {
                uri = String(uri.dropFirst(9))
            }

            let parts = uri.components(separatedBy: ":")
            guard parts.count == 2 else {
                return false
            }
            var port_str = parts[1]

            var segs = port_str.components(separatedBy: "/")
            port_str = segs[0]

            guard let port_nr = UInt16(port_str) else {
                return false
            }

            port = port_nr
            host = parts[0]
            return true
        }


        static func getTCPROSPort() -> UInt16 {
            return g_tcpros_server_port
        }

        static func determineHost() -> String {


            if let hostname = ProcessInfo.processInfo.environment["ROS_HOSTNAME"] {
                ROS_DEBUG("determineIP: using value of ROS_HOSTNAME:\(hostname)")
                if hostname.isEmpty {
                    ROS_WARNING("invalid ROS_HOSTNAME (an empty string)")
                }
                return hostname
            }

            if let rosIP = ProcessInfo.processInfo.environment["ROS_IP"] {
                ROS_DEBUG("determineIP: using value of ROS_IP:\(rosIP)")
                if rosIP.isEmpty {
                    ROS_WARNING("invalid ROS_IP (an empty string)")
                }
                return rosIP
            }

            let host = ProcessInfo.processInfo.hostName
            let trimmedHost = host.trimmingCharacters(in: .whitespacesAndNewlines)
            // We don't want localhost to be our ip
            if !trimmedHost.isEmpty && host != "localhost" && trimmedHost.contains(".") {
                return trimmedHost
            }


            for i in try! System.enumerateInterfaces() {
                let host = i.address.host
                if host != "127.0.0.1" && i.address.protocolFamily == PF_INET {
                    return host
                }
            }

            return "127.0.0.1"
        }
    }
}
