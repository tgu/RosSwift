//
//  names.swift
//  msgbuilder
//
//  Created by Thomas Gustafsson on 2019-04-29.
//

import Foundation


/// Check if name is a legal ROS name for filesystem resources
/// (alphabetical character followed by alphanumeric, underscore, or
/// forward slashes). This constraint is currently not being enforced,
/// but may start getting enforced in later versions of ROS.

func is_legal_resource_name(_ name: String) -> Bool {
    if name.isEmpty {
        return false
    }

    if let r = name.range(of: #"\b[A-Za-z][\w_\/]*\b"#, options: .regularExpression), name[r] == name {
        return !name.contains("//")
    }

    return false
}

func is_legal_resource_base_name(_ name: String) -> Bool {
    if name.isEmpty {
        return false
    }

    if let r = name.range(of: #"\b[A-Za-z][\w_]*\b"#, options: .regularExpression), name[r] == name {
        return true
    }

    return false
}
