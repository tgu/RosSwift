//
//  CallbackInterface.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-07.
//

enum CallResult {
    case success
    case tryAgain
    case invalid
}

protocol CallbackInterface: class {
    func call() -> CallResult
    func ready() -> Bool
}
