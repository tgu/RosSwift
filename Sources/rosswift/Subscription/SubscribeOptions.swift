//
//  SubscribeOptions.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-06.
//

import Foundation
import StdMsgs

public struct SubscribeOptions<M: Message> {
    var topic : String
//    var md5sum : String { return M.md5sum }
//    var datatype : String { return M.datatype }
    var transport_hints : TransportHints? = nil
    var helper: SubscriptionCallbackHelper? = nil
    var tracked_object: AnyObject? = nil
    var allow_concurrent_callbacks = false

    init(topic: String, callback: @escaping ((M) -> Void))
    {
        self.topic = topic
        self.helper = SubscriptionCallbackHelperT(callback: callback)
    }

}

