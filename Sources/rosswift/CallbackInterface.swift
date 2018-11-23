//
//  CallbackInterface.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-07.
//

public enum CallResult {
    case success
    case tryAgain
    case invalid
}

public protocol CallbackInterface: class {
    func call() -> CallResult
    func ready() -> Bool
}
