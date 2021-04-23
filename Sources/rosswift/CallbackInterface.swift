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

public protocol CallbackInterface {
    func call() -> CallResult
    
    /// Provides the opportunity for specifying that a callback is not ready to be called before call() actually takes place.
    var ready: Bool { get }
}
