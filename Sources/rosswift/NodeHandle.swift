//
//  NodeHandle.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation
import NIOConcurrencyHelpers
import StdMsgs

extension Ros {

    private static var nodeReferenceCount = Atomic<UInt>(value: 0)

    public final class NodeHandle {
        public var isOK: Bool { return Ros.isRunning && ok }

        private var namespace: String = "/"
        private var unresolvedNamespace = ""
        private var remappings = StringStringMap()
        private var unresolvedRemappings = StringStringMap()
        private var ok = false
        private var gNodeStartedByNodeHandle = false
        private var callbackQueue: CallbackQueueInterface?

        public init(ns: String = "", remappings: StringStringMap? = nil) {
            namespace = Ros.ThisNode.getNamespace()
            var tildeResolvedNameSpace = ns
            if ns.starts(with: "~") {
                tildeResolvedNameSpace = Names.resolve(name: ns)
            }
            construct(ns: tildeResolvedNameSpace, validateName: true)
            if let r = remappings {
                initRemappings(remappings: r)
            }
        }

        public init(parent: NodeHandle, ns: String = "", remappings: StringStringMap? = nil) {
            self.namespace = parent.namespace
            self.remappings = parent.remappings
            self.callbackQueue = parent.callbackQueue
            unresolvedRemappings = parent.unresolvedRemappings
            construct(ns: ns, validateName: false)
            if let r = remappings {
                initRemappings(remappings: r)
            }
        }

        deinit {
            destruct()
        }

        public func spinThread() {
            Ros.spin()
        }

        func destruct() {
            if Ros.nodeReferenceCount.sub(1) == 1 && gNodeStartedByNodeHandle {
                Ros.shutdown()
            }
        }

        func initRemappings(remappings: StringStringMap) {
            remappings.forEach {
                self.remappings[resolveName(name: $0.key, remap: false )] = resolveName(name: $0.value, remap: false)
                unresolvedRemappings[$0.key] = $0.value
            }
        }

        public func getCallbackQueue() -> CallbackQueueInterface {
            return callbackQueue != nil ? callbackQueue! : Ros.getGlobalCallbackQueue()
        }

        public func advertise<M: Message>(topic: String, latch: Bool = false, message: M.Type) -> Publisher? {
            let ops = AdvertiseOptions(topic: topic, latch: latch, M.self)
            return advertise(ops: ops)
        }

        public func advertise<M: Message>(ops: AdvertiseOptions<M>) -> Publisher? {
            var options = ops
            options.topic = resolveName(name: options.topic)
            if options.callbackQueue == nil {
                options.callbackQueue = getCallbackQueue()
            }
            let callbacks = SubscriberCallbacks(connect: options.connectCallBack,
                                                disconnect: options.disconnectCallBack,
                                                hasTrackedObject: options.trackedObject != nil,
                                                trackedObject: options.trackedObject)

            if TopicManager.instance.advertise(ops: options, callbacks: callbacks) {
                return SpecializedPublisher(topic: options.topic, message: M.self, node: self, callbacks: callbacks)
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
                let error = """
                    Using ~ names with NodeHandle methods is not allowed.
                    If you want to use private names with the NodeHandle interface,
                    construct a NodeHandle using private name as its namespace. e.g.
                        ros::NodeHandle nh(\"~\");
                        nh.getParam(\"my_private_name\" (name = [\(name)])
                    """
                fatalError(error)
            } else if final.starts(with: "/") {
                // do nothing
            } else if !namespace.isEmpty {
                final = Names.append(namespace, final)
            }

            final = Names.clean(final)

            if remap {
                final = remapName(name: final)
            }

            return Names.resolve(name: final, remap: false)

        }

        func remapName(name: String) -> String {
            let resolved = resolveName(name: name, remap: false)

            // First search any remappings that were passed in specifically for this NodeHandle
            if let it = remappings[resolved] {
                // ROSCPP_LOG_DEBUG("found 'local' remapping: %s", it->second.c_str());
                return it
            }

            // If not in our local remappings, perhaps in the global ones
            return Names.remapName(resolved)
        }

        func construct(ns: String, validateName: Bool) {
            if !isInitialized {
                fatalError("You must call Ros.initialize() before creating the first NodeHandle")
            }

            unresolvedNamespace = ns

            namespace = resolveName(name: ns, remap: true, validate: validateName)
            ok = true

            if Ros.nodeReferenceCount.add(1) == 0 && !isStarted {
                gNodeStartedByNodeHandle = true
                Ros.start()
            }
        }

        func setCallbackQueue(queue: CallbackQueueInterface) {
            callbackQueue = queue
        }

        public func subscribe<M: Message>(_ messageType: M.Type,
                                          topic: String,
                                          queueSize: UInt32,
                                          callback: @escaping (MessageEvent<M>) -> Void) -> Subscriber? {
            return subscribe(topic: topic, queueSize: queueSize, callback: callback)
        }

        public func subscribe<M: Message>(topic: String,
                                          queueSize: UInt32,
                                          callback: @escaping (MessageEvent<M>) -> Void,
                                          transportHints: TransportHints = TransportHints()) -> Subscriber? {
            var ops = SubscribeOptions(topic: topic, queueSize: queueSize, callback: callback)
            ops.transportHints = transportHints
            return subscribeWith(options: ops)
        }



        public func subscribe<M: Message>(_ messageType: M.Type,
                                          topic: String,
                                          queueSize: UInt32,
                                          callback: @escaping (M) -> Void) -> Subscriber? {
            return subscribe(topic: topic, queueSize: queueSize, callback: callback)
        }

        public func subscribe<M: Message>(topic: String,
                                          queueSize: UInt32,
                                          callback: @escaping (M) -> Void,
                                          transportHints: TransportHints = TransportHints()) -> Subscriber? {
            var ops = SubscribeOptions(topic: topic, queueSize: queueSize, callback: callback)
            ops.transportHints = transportHints
            return subscribeWith(options: ops)
        }

        public func subscribeWith<M: Message>(options: SubscribeOptions<M>) -> Subscriber? {
            var options = options
            options.topic = resolveName(name: options.topic)

            if TopicManager.instance.subscribeWith(options: options) {
                return Subscriber(topic: options.topic, node: self, helper: options.helper)
            }

            return nil
        }

        func serviceClient(service: String,
                           md5sum: String,
                           persistent: Bool = false,
                           headerValues: StringStringMap? = nil) -> ServiceClient {

            let name = resolveName(name: service)
            let client = ServiceClient(name: name, md5sum: md5sum, persistent: persistent, headerValues: headerValues)

            if !client.isValid() {
                ROS_ERROR("invalid client")
            }

            return client

        }

        public func advertiseService<MReq: ServiceMessage, MRes: ServiceMessage>(service: String,
                                                                                 srvFunc: @escaping (MReq) -> MRes?) -> ServiceServer? {
                let ops = AdvertiseServiceOptions(service: service, callback: srvFunc)
            return advertiseService(ops: ops, callback: srvFunc)
        }

        func advertiseService<MReq: ServiceMessage, MRes: ServiceMessage>(ops: AdvertiseServiceOptions<MReq, MRes>,
                                                                          callback: @escaping (MReq) -> MRes?) -> ServiceServer? {
            let options = ops
            options.service = resolveName(name: ops.service)
            if options.callbackQueue == nil {
                options.callbackQueue = getCallbackQueue()
            }
            if ServiceManager.instance.advertiseService(options) {
                let srv = ServiceServer(service: options.service, node: self)
                if srv.implementation == nil {
                    ROS_ERROR("invalid service server")
                }
                return srv
            }
            return nil
        }

        func setParam<T>(_ key: String, _ value: T) {
            Ros.Param.set(key, value)
        }

        func searchParam(key: String, result: inout String) -> Bool {
            // searchParam needs a separate form of remapping -- remapping on the unresolved name, rather than the
            // resolved one.

            var remapped = key
            if let it = unresolvedRemappings[key] {
                remapped = it
            }

            return Ros.Param.search(ns: resolveName(name: ""), key: remapped, result: &result)
        }

        func deleteParam(_ key: String) -> Bool {
            return Ros.Param.del(key: resolveName(name: key))
        }

        func hasParam(_ key: String) -> Bool {
            return Ros.Param.has(key: resolveName(name: key))
        }

        func getParam<T>(name: String, value: inout T) -> Bool {
            return Ros.Param.get(name, &value)
        }

        func param<T>(name: String, value: inout T, defaultValue: T) -> Bool {
            if hasParam(name) {
                if getParam(name: name, value: &value) {
                    return true
                }
            }

            value = defaultValue
            return false
        }

        func param<T>(name: String, defaultValue: T) -> T {
            var paramVal = defaultValue
            _ = param(name: name, value: &paramVal, defaultValue: defaultValue)
            return paramVal
        }

    }

}
