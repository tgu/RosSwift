
//
//  File.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation
import StdMsgs
import BinaryCoder

enum InvalidParameterError: Error {
    case invalidParameter(String)

    init(_ str: String) {
        self = .invalidParameter(str)
    }
}

func md5sumsMatch(lhs: String, rhs: String) -> Bool {
    return lhs == "*" || rhs == "*" || lhs == rhs
}

extension Ros {


    final class TopicManager {

        static let instance = TopicManager()

        let advertised_topics_mutex_ = DispatchQueue(label: "advertised_topics_mutex_")
        let subs_mutex_ = DispatchQueue(label: "subs_mutex_")
        let shutting_down_mutex_ = DispatchQueue(label: "shutting_down_mutex_")

        var advertisedTopics = [Publication]()
        var subscriptions = [Subscription]()
        var advertised_topic_names_ = SynchronizedArray<String>()

        var shutting_down = false

        var connection_manager: ConnectionManager {
            return Ros.ConnectionManager.instance
        }

        var xmlrpc_manager: XMLRPCManager {
            return XMLRPCManager.instance
        }

        private init() {}

        deinit {
            shutdown()
        }

        func start() {
            shutting_down_mutex_.sync {
                shutting_down = false

                let _ = xmlrpc_manager.bind(function_name: "publisherUpdate", cb: pubUpdateCallback)
                let _ = xmlrpc_manager.bind(function_name: "requestTopic", cb: requestTopicCallback)
                let _ = xmlrpc_manager.bind(function_name: "getBusStats", cb: getBusStatsCallback)
                let _ = xmlrpc_manager.bind(function_name: "getBusInfo", cb: getBusInfoCallback)
                let _ = xmlrpc_manager.bind(function_name: "getSubscriptions", cb: getSubscriptionsCallback)
                let _ = xmlrpc_manager.bind(function_name: "getPublications", cb: getPublicationsCallback)

            }
        }

        public func shutdown() {
            shutting_down_mutex_.sync {
                if shutting_down {
                    return
                }

                advertised_topics_mutex_.sync {
                    subs_mutex_.sync {
                        shutting_down = true
                    }
                }

                xmlrpc_manager.unbind(function_name: "publisherUpdate")
                xmlrpc_manager.unbind(function_name: "requestTopic")
                xmlrpc_manager.unbind(function_name: "getBusStats")
                xmlrpc_manager.unbind(function_name: "getBusInfo")
                xmlrpc_manager.unbind(function_name: "getSubscriptions")
                xmlrpc_manager.unbind(function_name: "getPublications")

                ROS_DEBUG("Shutting down topics...")
                ROS_DEBUG("shutting down publishers")

                advertised_topics_mutex_.sync {
                    advertisedTopics.forEach {
                        if !$0.dropped.load() {
                            ROS_DEBUG("shutting down \($0.name)")
                            let _ = unregisterPublisher(topic: $0.name)
                        }
                        $0.drop()
                    }
                    advertisedTopics.removeAll()
                }

                ROS_DEBUG("shutting down subscribers we have \(subscriptions.count) subscriptions")
                subs_mutex_.sync {

                    subscriptions.forEach {
                        ROS_DEBUG("closing down subscription to \($0.name)")
                        let _ = unregisterSubscriber(topic: $0.name)
                        $0.shutdown()
                    }
                    subscriptions.removeAll()
                }

            }
        }


        func pubUpdateCallback(params: XmlRpcValue) -> XmlRpcValue {
            var pubs = [String]()
            for idx in 0..<params[2].size() {
                pubs.append(params[2][idx].string)
            }
            if pubUpdate(topic: params[1].string, pubs: pubs) {
                return XmlRpcValue(anyArray: [1,"",0])
            } else {
                return XmlRpcValue(anyArray: [0,"last error",0])
            }
        }

        func requestTopicCallback(params: XmlRpcValue) -> XmlRpcValue {
            return requestTopic(topic: params[1].string, protos: params[2])
        }

        func requestTopic(topic: String, protos: XmlRpcValue) -> XmlRpcValue {
            ROS_DEBUG("requestTopic \(topic) with \(protos)")
            for proto_idx in 0..<protos.size() {
                let proto = protos[proto_idx]
                guard case .array = proto.value else {
                    ROS_DEBUG("requestTopic protocol list was not a list of lists")
                    return XmlRpcValue()
                }

                guard case .string(let proto_name) = proto[0].value else {
                    ROS_DEBUG( "requestTopic received a protocol list in which a sublist did not start with a string")
                    return XmlRpcValue()
                }

                if proto_name == "TCPROS" {
                    let tcpros_params = XmlRpcValue(array: [.init(str: "TCPROS"),
                                                            .init(str: network.getHost()),
                                                            .init(any: Ros.ConnectionManager.instance.getTCPPort())])
                    let ret = XmlRpcValue(array: [.init(any: 1),
                                                  .init(str: ""),
                                                  tcpros_params])
                    return ret
                } else {
                    ROS_DEBUG( "an unsupported protocol was offered: [\(proto_name)]")
                }
            }
            return XmlRpcValue()
        }

        func getBusStatsCallback(params: XmlRpcValue) -> XmlRpcValue {
            var response = XmlRpcValue()
            getBusStats(stats: &response)
            return XmlRpcValue(anyArray: [1,"",response])
        }

        func getBusInfoCallback(params: XmlRpcValue ) -> XmlRpcValue {
            var response = XmlRpcValue()
            getBusInfo(info: &response)
            return XmlRpcValue(anyArray: [1,"",response])
        }

        func getSubscriptionsCallback(params: XmlRpcValue ) -> XmlRpcValue {
            var response = XmlRpcValue()
            getSubscriptions(subs: &response)
            return XmlRpcValue(anyArray: [1,"",response])
        }

        func getPublicationsCallback(params: XmlRpcValue ) -> XmlRpcValue {
            var response = XmlRpcValue()
            getPublications(pubs: &response)
            return XmlRpcValue(anyArray: [1,"",response])
        }

        func getBusStats(stats: inout XmlRpcValue) {
            ROS_ERROR("getBusStats not implemented")
        }

        func getBusInfo(info: inout XmlRpcValue) {
            var i = [XmlRpcValue]()
            advertised_topics_mutex_.sync {
                advertisedTopics.forEach({ (pub) in
                    i.append(contentsOf: pub.getInfo() )
                })
            }

            subs_mutex_.sync {
                subscriptions.forEach({ (sub) in
                    i.append(contentsOf: sub.getInfo() )
                })
            }

            info.value = .array(i)
        }

        func getSubscriptions(subs: inout XmlRpcValue) {
            ROS_ERROR("getSubscriptions not implemented")
        }

        func getPublications(pubs: inout XmlRpcValue) {
            let topics = advertisedTopics.map { (pub) -> XmlRpcValue in
                return XmlRpcValue(anyArray: [pub.name,pub.datatype])
            }
            pubs = XmlRpcValue(anyArray: topics)
        }


        func pubUpdate(topic: String, pubs: [String]) -> Bool {
            let sub : Subscription? = subs_mutex_.sync {
                if shutting_down {
                    return nil
                }
                ROS_DEBUG("Received update for topic [\(topic)] (\(pubs.count) publishers)")
                return subscriptions.first(where: { (s) -> Bool in
                    return !s.dropped.load() && s.name == topic
                })
            }

            if let s = sub {
                return s.pubUpdate(new_pubs: pubs)
            } else {
                ROS_DEBUG("got a request for updating publishers of topic \(topic), but I don't have any subscribers to that topic.")

            }

            return false
        }

        func lookupPublication(topic: String) -> Publication? {
            return advertised_topics_mutex_.sync {
                return lookupPublicationWithoutLock(topic: topic)
            }
        }

        func getAdvertised(topics: inout [String]) {
            advertised_topics_mutex_.sync {
                topics = advertised_topic_names_.all()
            }
        }

        func getSubscribed(topics: inout [String]) {
            subs_mutex_.sync {
                topics = subscriptions.map { $0.name }
            }
        }



        func unregisterPublisher(topic: String) -> Bool {
            ROS_DEBUG("unregister publisher \(topic)")
            let args = XmlRpcValue(anyArray: [Ros.this_node.getName(),topic,XMLRPCManager.instance.serverURI])
            do {
                let response = try Master.shared.execute(method: "unregisterPublisher", request: args).wait()
                ROS_DEBUG("response = \(response)")
            } catch {
                ROS_ERROR("Error during unregisterPublisher \(error)")
            }

            return true
        }

        /** if it finds a pre-existing subscription to the same topic and of the
         *  same message type, it appends the Functor to the callback vector for
         *  that subscription. otherwise, it returns false, indicating that a new
         *  subscription needs to be created.
         */

        func addSubCallback<M: Message>(ops: SubscribeOptions<M>) -> Bool {
            // spin through the subscriptions and see if we find a match. if so, use it.
            var found = false

            var sub : Subscription? = nil

            if shutting_down {
                return false
            }

            subscriptions.forEach {
                if !$0.dropped.load() && $0.name == ops.topic {
                    if md5sumsMatch(lhs: M.md5sum, rhs: $0.md5sum_) {
                        found = true
                    }
                    sub = $0
                    return
                }
            }


            if let sub = sub {
                if !found {
                    fatalError("Tried to subscribe to a topic with the same name but different md5sum as a topic that was already subscribed [\(M.datatype)/\(M.md5sum) vs. \(sub.datatype)/\(sub.md5sum_)]")
                } else if !sub.add(callback: ops.helper!, md5sum: M.md5sum, tracked_object: ops.tracked_object, allow_concurrent_callbacks: ops.allow_concurrent_callbacks) {
                    return false
                }
            }
            return found

        }


        func subscribeWith<M: Message>(options: SubscribeOptions<M>) -> Bool {
            var ok = true
            do {
                try subs_mutex_.sync {

                    if addSubCallback(ops: options) {
                        return
                    }

                    if shutting_down {
                        ok = false
                    }

                    if M.md5sum == "" {
                        throw InvalidParameterError("Subscribing to topic [\(options.topic)] with an empty md5sum");
                    }

                    if M.datatype == "" {
                        throw InvalidParameterError("Subscribing to topic [\(options.topic)] with an empty datatype");
                    }

                    if options.helper == nil {
                        throw InvalidParameterError("Subscribing to topic [\(options.topic)] without a callback");
                    }

                    let md5sum = M.md5sum
                    let datatype = M.datatype

                    let s = Subscription(name: options.topic, md5sum: md5sum, datatype: datatype, transport_hints: options.transport_hints!)
                    let _ = s.add(callback: options.helper!, md5sum: M.md5sum, tracked_object: options.tracked_object, allow_concurrent_callbacks: options.allow_concurrent_callbacks)

                    if !registerSubscriber(s: s, datatype: M.datatype) {
                        ROS_DEBUG("couldn't register subscriber on topic [\(options.topic)")
                        s.shutdown()
                        ok = false
                        return
                    }

                    subscriptions.append(s)
                }
            } catch {
                ROS_ERROR(error.localizedDescription)
                return false
            }
            return ok
        }


        func unsubscribe(topic: String, helper: SubscriptionCallbackHelper) -> Bool {
            var sub : Subscription?
            subs_mutex_.sync {
                if !shutting_down {
                    sub = subscriptions.first(where: {$0.name == topic})
                }
            }

            guard let s = sub else {
                return false
            }

            s.remove(callback: helper)

            if s.callbacks.count == 0 {
                // nobody is left. blow away the subscription.
                subs_mutex_.sync {
                    subscriptions = subscriptions.filter { $0.name != topic }
                    unregisterSubscriber(topic: topic)
                }

                s.shutdown()
            }

            return true
        }


        func registerSubscriber(s: Subscription, datatype: String) -> Bool {
            let args = XmlRpcValue(anyArray: [Ros.this_node.getName(),s.name,datatype,XMLRPCManager.instance.serverURI])

            var payload = XmlRpcValue()
            do {
                payload = try Master.shared.execute(method: "registerSubscriber", request: args).wait()
            } catch {
                ROS_ERROR("registerSubscriber \(error)")
            }

            guard payload.valid() else {
                return false
            }

            var pub_uris = [String]()
            for i in 0..<payload.size() {
                if case .string(let uri) = payload[i].value {
                    if uri != XMLRPCManager.instance.serverURI {
                        if !uri.isEmpty {
                            pub_uris.append(uri)
                        } else {
                            ROS_DEBUG("empty public uri")
                        }
                    }
                }
            }

            var pub_local : Publication? = nil
            var self_subscribed = false
            //        var pub = Publication()
            let sub_md5sum = s.md5sum_
            // Figure out if we have a local publisher

            var ok = true
            advertised_topics_mutex_.sync {
                for pub in advertisedTopics {
                    let pub_md5sum = pub.md5sum

                    if pub.name == s.name && !pub.dropped.load() {
                        if !md5sumsMatch(lhs: pub_md5sum, rhs: sub_md5sum) {
                            ROS_DEBUG("md5sum mismatch making local subscription to topic \(s.name).")
                            ROS_DEBUG("Subscriber expects type \(s.datatype), md5sum \(s.md5sum_)")
                            ROS_DEBUG("Publisher provides type \(pub.datatype), md5sum \(pub.md5sum)")
                            ok = false
                        } else {
                            self_subscribed = true
                            pub_local = pub
                        }
                        break
                    }
                }
            }

            if ok {
                let _ = s.pubUpdate(new_pubs: pub_uris)
                if self_subscribed, let pl = pub_local {
                    s.add(localConnection: pl)
                }
            }

            return ok
        }

        func unregisterSubscriber(topic: String)  {
            let args = XmlRpcValue(anyArray: [Ros.this_node.getName(),topic,xmlrpc_manager.serverURI])
            let response = Master.shared.execute(method: "unregisterSubscriber", request: args)
            response.whenFailure { (error) in
                ROS_ERROR("ouldn't unregister subscriber for topic [\(topic)]: \(error)")
            }
        }


        func lookupPublicationWithoutLock(topic: String) -> Publication? {
            return advertisedTopics.first { $0.name == topic }
        }


        func advertise<M: Message>(ops: AdvertiseOptions<M>, callbacks: SubscriberCallbacks) -> Bool {
            if M.datatype == "*" {
                fatalError("Advertising with * as the datatype is not allowed.  Topic [\(ops.topic)")
            }
            if M.md5sum == "*" {
                fatalError("Advertising with * as the md5sum is not allowed.  Topic [\(ops.topic)]")
            }
            if M.md5sum.isEmpty {
                fatalError("Advertising on topic [\(ops.topic)] with an empty md5sum")
            }
            if M.datatype.isEmpty {
                fatalError("Advertising on topic [\(ops.topic)] with an empty datatype")
            }
            if M.definition.isEmpty {
                ROS_DEBUG("Advertising on topic \(ops.topic) with an empty message definition.  Some tools (e.g. rosbag) may not work correctly.")
            }


            if let pub = lookupPublicationWithoutLock(topic: ops.topic) {
                if pub.numCallbacks == 0 {
                    //                pub.reset()
                }
                if pub.md5sum != M.md5sum {
                    ROS_DEBUG("wrong md5sum")
                    return false
                }

                pub.addCallbacks(callback: callbacks)

                return true
            }

            let pub = Publication(name: ops.topic, datatype: M.datatype, md5sum: M.md5sum, message_definition: M.definition, latch: ops.latch, has_header: M.hasHeader)
            pub.addCallbacks(callback: callbacks)
            advertisedTopics.append(pub)

            advertised_topic_names_.append(ops.topic)

            // Check whether we've already subscribed to this topic.  If so, we'll do
            // the self-subscription here, to avoid the deadlock that would happen if
            // the ROS thread later got the publisherUpdate with its own XMLRPC URI.
            // The assumption is that advertise() is called from somewhere other
            // than the ROS thread.

            subs_mutex_.sync                 {
                if let it = subscriptions.first(where: { (s) -> Bool in
                    s.name == ops.topic && md5sumsMatch(lhs: s.md5sum_, rhs: M.md5sum) && !s.dropped.load()
                }) {
                    DispatchQueue.main.async {
                        it.add(localConnection: pub)
                    }
                }
            }


            let args = XmlRpcValue(array: [.init(str: Ros.this_node.getName()),
                                           .init(str: ops.topic),
                                           .init(str: M.datatype),
                                           .init(str: XMLRPCManager.instance.serverURI)])
            var payload = XmlRpcValue()
            do {
                payload = try Master.shared.execute(method: "registerPublisher", request: args).wait()
            } catch {
                ROS_ERROR("registerPublisher \(error)")
            }
            ROS_DEBUG(payload.description)

            return true
        }

        func unadvertisePublisher(topic: String, callbacks: SubscriberCallbacks?) -> Bool {
            var pub : Publication?
            advertised_topics_mutex_.sync {
                if !shutting_down {
                    pub = advertisedTopics.first(where: { $0.name == topic && !$0.dropped.load() })
                }
            }

            guard let p = pub else {
                return false
            }

            if let c = callbacks {
                p.removeCallbacks(callback: c)
            }

            advertised_topics_mutex_.sync {
                if p.numCallbacks == 0 {
                    unregisterPublisher(topic: p.name)
                    p.drop()
                    #if swift(>=4.2)
                    advertisedTopics.removeAll(where: { $0.name == topic && !$0.dropped })
                    #else
                    ROS_DEBUG("advertisedTopics.removeAll not called")
                    #endif
                    advertised_topic_names_.remove(where: { $0 == topic })
                }
            }

            return true
        }

        func publish(topic: String, ser_msg: SerializedMessage) {
            if shutting_down {
                return
            }

            advertised_topics_mutex_.sync {

                if let p = lookupPublicationWithoutLock(topic: topic) {

                    if p.hasSubscribers() || p.isLatching() {
                        ROS_DEBUG("Publishing message on topic [\(p.name)] with sequence number [\(p.sequenceNr.load())]")

                        p.publish(msg: ser_msg)

                    } else {
                        let _ = p.incrementSequence()
                    }
                }
            }
        }


        func incrementSequence(topic: String) {
            let _ = lookupPublication(topic: topic)?.incrementSequence()
        }

        func isLatched(topic: String) -> Bool {
            if let pub = lookupPublication(topic: topic) {
                return pub.isLatched()
            }
            return false
        }

        func getNumPublishers(topic: String) -> Int {
            if shutting_down {
                return 0
            }

            return subs_mutex_.sync {
                if let sub = subscriptions.first(where: { (sub) -> Bool in
                    return !sub.dropped.load() && sub.name == topic
                }) {
                    return sub.getNumPublishers()
                }
                return 0
            }
        }

        func getNumSubscribers(topic: String) -> Int {
            if shutting_down {
                return 0
            }

            return advertised_topics_mutex_.sync {
                if let p = lookupPublicationWithoutLock(topic: topic) {
                    return p.getNumSubscribers()
                } else {
                    return 0
                }
            }
        }



        func getNumSubscriptions() -> Int {
            return subs_mutex_.sync {
                return subscriptions.count
            }
        }

    }

}
