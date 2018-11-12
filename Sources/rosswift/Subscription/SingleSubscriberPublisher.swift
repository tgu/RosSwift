//
//  SingleSubscriberPublisher.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation
import StdMsgs
import BinaryCoder
 

public class SingleSubscriberPublisher {
    var link: SubscriberLink

    init(link: SubscriberLink) {
        self.link = link
    }


    public func publish<M: Message>(_ msg: M) {
        do {
            let data = try BinaryEncoder.encode(msg)
            var count = try BinaryEncoder.encode(UInt32(data.count))
            count.append(contentsOf: data)
            let ser = SerializedMessage(msg: msg, buffer: count)
            publish(m: ser)
        } catch {
            ROS_ERROR("publish failed \(error)")
        }
    }

    func publish(m: StdMsgs.SerializedMessage) {
        link.enqueueMessage(m: m, ser: true, nocopy: true)
    }

}
