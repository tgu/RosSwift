//
//  network.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation
import NIO

    internal struct Network {
        private let host: String
        private let gTcprosServerPort: UInt16

        func getHost() -> String {
            return host
        }

        init(remappings: StringStringMap) {
            var gHost = ""
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
            } else {
                gTcprosServerPort = 0
            }

            if gHost.isEmpty {
                gHost = Network.determineHost()
            }
            host = gHost
        }

        static func splitURI(uri: String) -> (host: String, port: UInt16)? {
            var uri = uri
            if uri.hasPrefix("http://") {
                uri = String(uri.dropFirst(7))
            } else if uri.hasPrefix("rosrpc://") {
                uri = String(uri.dropFirst(9))
            }

            let parts = uri.components(separatedBy: ":")
            guard parts.count == 2 else {
                return nil
            }
            var portString = parts[1]

            var segs = portString.components(separatedBy: "/")
            portString = segs[0]

            guard let portNr = UInt16(portString) else {
                return nil
            }

            return (parts[0], portNr)
        }

        func getTCPROSPort() -> UInt16 {
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
