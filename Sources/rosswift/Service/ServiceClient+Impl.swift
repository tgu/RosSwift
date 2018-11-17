//
//  ServiceClient+Impl.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-25.
//

import Foundation

extension ServiceClient {

    internal final class Impl {
        var serverLink: ServiceServerLink?
        let name: String
        let persistent: Bool
        let headerValue: StringStringMap?
        let serviceMd5sum: String
        var isShutdown: Bool = false

        init(name: String, md5sum: String, persistent: Bool, header: StringStringMap?) {
            self.name = name
            self.persistent = persistent
            headerValue = header
            serviceMd5sum = md5sum
        }

        deinit {
            shutdown()
        }

        func shutdown() {
            if !isShutdown {
                if !persistent {
                    isShutdown = true
                }
                serverLink = nil
            }
        }

        func isValid() -> Bool {
            if !persistent {
                return true
            }
            if isShutdown {
                return false
            }
            if let link = serverLink {
                return link.isValid()
            }
            return false
        }
    }

}
