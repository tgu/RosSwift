//
//  ServiceCallBackHelper.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-10.
//

import Foundation
import StdMsgs

struct ServiceSpecCallParams {
    var request: Message
    var response: Message
    var connectionHeader: StringStringMap
}

struct ServiceCallbackHelperCallParams {
    let request: SerializedMessage
    let response: SerializedMessage
    let connectionHeader: StringStringMap
}

protocol ServiceCallbackHelper {

    func call(params: ServiceCallbackHelperCallParams) -> Bool

}

typealias ReqCreateFunction = () -> Message
typealias ResCreateFunction = () -> Message

final class ServiceCallbackHelperT: ServiceCallbackHelper {

    let callback: Callback
    let createRequest: ReqCreateFunction
    let createResponse: ResCreateFunction

    init(callback: @escaping Callback,
         createRequest: @escaping ReqCreateFunction,
         createResponse: @escaping ResCreateFunction ) {

        self.callback = callback
        self.createRequest = createRequest
        self.createResponse = createResponse
    }

    func call(params: ServiceCallbackHelperCallParams) -> Bool {
        let req = createRequest()
        let res = createResponse()

        _ = ServiceSpecCallParams(request: req, response: res, connectionHeader: params.connectionHeader)

        ROS_ERROR("ServiceCallbackHelperT.call not implemented")
        return false
    }

}
