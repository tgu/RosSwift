//
//  network.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation
import NIO

extension Ros {

    struct Network {
        static var gHost = ""
        static var gTcprosServerPort: UInt16 = 0

        static func getHost() -> String {
            return gHost
        }

        static func initialize(remappings: StringStringMap) {
            if let it = remappings["__hostname"] {
                gHost = it
            } else if let it = remappings["__ip"] {
                gHost = it
            }
            if let it = remappings["__tcpros_server_port"] {
                guard let tcprosServerPort = UInt16(it) else {
                    fatalError("__tcpros_server_port [\(it)] was not specified as a number within the 0-65535 range")
                }
                gTcprosServerPort = tcprosServerPort
            }

            if gHost.isEmpty {
                gHost = determineHost()
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
            var portString = parts[1]

            var segs = portString.components(separatedBy: "/")
            portString = segs[0]

            guard let portNr = UInt16(portString) else {
                return false
            }

            port = portNr
            host = parts[0]
            return true
        }

        static func getTCPROSPort() -> UInt16 {
            return gTcprosServerPort
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

            do {
                for i in try System.enumerateInterfaces() {
                    let host = i.address.host
                    if host != "127.0.0.1" && i.address.protocolFamily == PF_INET {
                        return host
                    }
                }
            } catch {
                ROS_ERROR(error.localizedDescription)
            }

            return "127.0.0.1"
        }
    }
}
