//
//  ServiceClient+Impl.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-25.
//

import Foundation

extension ServiceClient {

    internal class Impl {
        var server_link_ : ServiceServerLink? = nil
        let name_ : String
        let persistent_ : Bool
        let header_value_ : M_string?
        let service_md5sum_ : String
        var is_shutdown_ : Bool = false

        init(name: String, md5sum: String, persistent: Bool, header: M_string?) {
            name_ = name
            persistent_ = persistent
            header_value_ = header
            service_md5sum_ = md5sum
        }

        deinit {
            shutdown()
        }

        func shutdown() {
            if !is_shutdown_ {
                if !persistent_ {
                    is_shutdown_ = true
                }
                if let sl = server_link_ {
//                    sl.getConnection()?.drop(reason: .Destructing)
                    server_link_ = nil
                }
            }

        }

        func isValid() -> Bool {
            if !persistent_ {
                return true
            }
            if is_shutdown_ {
                return false
            }
            if let sl = server_link_ {
                return sl.isValid()
            }
            return false
        }
    }


}
