//
//  ServiceCallBackHelper.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-10.
//

import Foundation
import StdMsgs


struct ServiceSpecCallParams
{
    var request : Message
    var response : Message
    var connection_header : M_string
}

struct ServiceCallbackHelperCallParams {
    let request : SerializedMessage
    let response : SerializedMessage
    let connection_header : M_string
}

protocol ServiceCallbackHelper {

    func call(params: ServiceCallbackHelperCallParams) -> Bool

}

typealias ReqCreateFunction = () -> Message
typealias ResCreateFunction = () -> Message

final class ServiceCallbackHelperT: ServiceCallbackHelper {

    let callback_ : Callback
    let create_req_ : ReqCreateFunction
    let create_res_ : ResCreateFunction

    init(callback: @escaping Callback, create_req: @escaping ReqCreateFunction, create_res: @escaping ResCreateFunction ) {

        self.callback_  = callback
        self.create_req_ = create_req
        self.create_res_ = create_res
    }



    func call(params: ServiceCallbackHelperCallParams) -> Bool {
        let req = create_req_()
        let res = create_res_()


        _ = ServiceSpecCallParams(request: req, response: res, connection_header: params.connection_header)

        ROS_ERROR("ServiceCallbackHelperT.call not implemented")
        return false
    }


}
