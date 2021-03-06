//
//  XmlRpcServerMethod.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import rpcobject

protocol XmlRpcServerMethod {
    var name: String { get }

    func execute(params: XmlRpcValue) throws -> XmlRpcValue
    func help() -> String
}
