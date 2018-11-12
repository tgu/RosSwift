//
//  NodeHandle.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation
import StdMsgs
import NIOConcurrencyHelpers


extension Ros {

    private static var nodeReferenceCount = Atomic<UInt>(value: 0)

    public class NodeHandle {
        public var ok : Bool { return Ros.ok && ok_ }

        private var namespace: String = "/"
        private var unresolvedNamespace = ""
        private var remappings = M_string()
        private var unresolvedRemappings = M_string()
        private var ok_ = false
        private var g_node_started_by_nh = false

        public init(ns: String = "", remappings: M_string? = nil) {
            namespace = Ros.this_node.getNamespace()
            var tilde_resolved_ns = ns
            if ns.starts(with: "~") {
                tilde_resolved_ns = Names.resolve(name: ns)
            }
            construct(ns: tilde_resolved_ns, validate_name: true)
            if let r = remappings {
                initRemappings(remappings: r)
            }
        }

        public init(parent: NodeHandle, ns: String = "", remappings: M_string? = nil) {
            self.namespace = parent.namespace
            self.remappings = parent.remappings
            unresolvedRemappings = parent.unresolvedRemappings
            construct(ns: ns, validate_name: false)
            if let r = remappings {
                initRemappings(remappings: r)
            }
        }

        deinit {
            descruct()
        }

        func descruct() {
            if Ros.nodeReferenceCount.sub(1) == 1 && g_node_started_by_nh {
                Ros.shutdown()
            }
        }

        func initRemappings(remappings: M_string) {
            remappings.forEach {
                self.remappings[resolveName(name: $0.key, remap: false )] = resolveName(name: $0.value, remap: false)
                unresolvedRemappings[$0.key] = $0.value
            }
        }

        public func advertise<M: Message>(topic: String, latch: Bool = false, message: M.Type) -> Publisher? {
            let ops = AdvertiseOptions(topic: topic, latch: latch, M.self)
            return advertise(ops: ops)
        }

        public func advertise<M: Message>(ops: AdvertiseOptions<M>) -> Publisher? {
            var options = ops
            options.topic = resolveName(name: options.topic)
            let callbacks = SubscriberCallbacks(connect: options.connect_cb, disconnect: options.disconnect_cb, has_tracked_object: options.tracked_object != nil, tracked_object: options.tracked_object)

            if TopicManager.instance.advertise(ops: options, callbacks: callbacks) {
                return SpecializedPublisher(topic: options.topic, message: M.self, node_handle: self, callbacks: callbacks)
            }

            return nil
        }

        func resolveName(name: String, remap: Bool = true, validate: Bool = true) -> String {
            var error = ""
            if validate && !Names.validate(name: name, error: &error) {
                fatalError(error)
            }

            if name.isEmpty {
                return namespace
            }

            var final = name
            if final.starts(with: "~") {
                let ss = "Using ~ names with NodeHandle methods is not allowed.  If you want to use private names with the NodeHandle interface, construct a NodeHandle using a private name as its namespace.  e.g. ros::NodeHandle nh(\"~\");  nh.getParam(\"my_private_name\" (name = [\(name)])"
                fatalError(ss)
            } else if final.starts(with: "/") {
                // do nothing
            } else if !namespace.isEmpty {
                final = Names.append(namespace,final)
            }

            final = Names.clean(final)

            if remap {
                final = remapName(name: final)
            }

            return Names.resolve(name: final, _remap: false)

        }

        func remapName(name: String) -> String {
            let resolved = resolveName(name: name, remap: false)

            // First search any remappings that were passed in specifically for this NodeHandle
            if let it = remappings[resolved] {
                // ROSCPP_LOG_DEBUG("found 'local' remapping: %s", it->second.c_str());
                return it
            }


            // If not in our local remappings, perhaps in the global ones
            return Names.remap(resolved)
        }


        func construct(ns: String, validate_name: Bool) {
            if !isInitialized {
                fatalError("You must call Ros.initialize() before creating the first NodeHandle");
            }

            unresolvedNamespace = ns

            namespace = resolveName(name: ns, remap: true, validate: validate_name)
            ok_ = true

            if Ros.nodeReferenceCount.add(1) == 0 && !isStarted {
                g_node_started_by_nh = true
                Ros.start()
            }
        }

        public func subscribe<M: Message>(_ messageType: M.Type, topic: String, callback: @escaping (M) -> Void) -> Subscriber? {
            return subscribe(topic: topic, callback: callback)
        }


        public func subscribe<M: Message>(topic: String, callback: @escaping (M) -> Void, transport_hints:
        TransportHints = TransportHints()) -> Subscriber? {
            var ops = SubscribeOptions(topic: topic, callback: callback)
            ops.transport_hints = transport_hints
            return subscribeWith(options: ops)
        }

        public func subscribeWith<M: Message>(options: SubscribeOptions<M>) -> Subscriber? {
            var options_ = options
            options_.topic = resolveName(name: options_.topic)

            if TopicManager.instance.subscribeWith(options: options_) {
                return Subscriber(topic: options_.topic, node_handle: self, helper: options_.helper)
            }

            return nil
        }


        func serviceClient(service: String, md5sum: String, persistent: Bool = false, header_values: M_string? = nil) -> ServiceClient {
            let name = resolveName(name: service)
            let client = ServiceClient(name: name, md5sum: md5sum, persistent: persistent, header_values: header_values)

            if !client.isValid() {
                ROS_ERROR("invalid client")
            }

            return client

        }

        public func advertiseService<MReq: ServiceMessage, MRes: ServiceMessage>(service: String, srv_func: @escaping (MReq) -> MRes?) -> ServiceServer? {
                let ops = AdvertiseServiceOptions(service: service, callback: srv_func)
            return advertiseService(ops: ops, callback: srv_func)
        }

        func advertiseService<MReq: ServiceMessage, MRes: ServiceMessage>(ops: AdvertiseServiceOptions<MReq, MRes>, callback: @escaping (MReq) -> MRes?) -> ServiceServer? {
            let o = ops
            o.service = resolveName(name: ops.service)
            if ServiceManager.instance.advertiseService(o) {
                let srv = ServiceServer(service: o.service, node_handle: self)
                if srv.impl_ == nil {
                    ROS_ERROR("invalid service server")
                }
                return srv
            }
            return nil
        }

        func setParam<T>(_ key: String, _ value: T) {
            Ros.param.set(key, value)
        }

        func searchParam(key: String, result_out: inout String) -> Bool {
            // searchParam needs a separate form of remapping -- remapping on the unresolved name, rather than the
            // resolved one.

            var remapped = key
            if let it = unresolvedRemappings[key] {
                remapped = it
            }

            return Ros.param.search(ns: resolveName(name: ""), key: remapped, result: &result_out)
        }

        func deleteParam(_ key: String) -> Bool  {
            return Ros.param.del(key: resolveName(name: key))
        }

        func hasParam(_ key: String) -> Bool {
            return Ros.param.has(key: resolveName(name: key))
        }

        func getParam<T>(name: String, value: inout T) -> Bool {
            return Ros.param.get(name, &value)
        }

        func param<T>(param_name: String, param_val: inout T, default_val: T) -> Bool {
            if hasParam(param_name) {
                if getParam(name: param_name, value: &param_val) {
                    return true
                }
            }

            param_val = default_val
            return false
        }

        func param<T>(param_name: String, default_val: T) -> T {
            var param_val = default_val
            param(param_name: param_name, param_val: &param_val, default_val: default_val)
            return param_val
        }


    }

}
