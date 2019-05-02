//
//  NodeHandle.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation
import NIOConcurrencyHelpers
import StdMsgs
import RosTime

public final class NodeHandle {

    /// Check whether it's time to exit.
    ///
    /// This method checks to see if both Ros.ok() is true and shutdown()
    /// has not been called on this NodeHandle, to see whether it's yet time to exit.
    /// `ok` is false once either Ros.shutdown() or NodeHandle.shutdown() have been called

    public var isOK: Bool { return ros.isRunning && ok }

    ///  the namespace associated with this NodeHandle.
    public private(set) var namespace: String = "/"

    /// the namespace associated with this NodeHandle as it was passed in (before it was resolved)
    public private(set) var unresolvedNamespace = ""

    public private(set) var remappings = StringStringMap()

    public private(set) var unresolvedRemappings = StringStringMap()


    public private(set) var ok = true

    public private(set) var gNodeStartedByNodeHandle = false
    private var callbackQueue: CallbackQueueInterface?

    public var refCount: UInt {
        return ros.nodeReferenceCount.load()
    }

    internal let ros: Ros

    // MARK: Life

    //        internal init(ros: Ros) {
    //            self.ros = ros
    //            namespace = ros.namespace
    //            construct(ns: "")
    //        }


    /// Constructor.
    ///
    /// When a NodeHandle is constructed, it checks to see if the global node state has already been
    /// started. If so, it increments a global reference count. If not, it starts the node with
    /// ros::start() and sets the reference count to 1.
    ///
    /// - Parameters:
    ///     - ns:    Namespace for this NodeHandle. This acts in addition to any namespace assigned to
    /// this ROS node. eg. If the node's namespace is "/a" and the namespace passed in here is "b",
    /// all topics/services/parameters will be prefixed with "/a/b/"
    ///     - remappings:    Remappings for this NodeHandle.

    internal init?(ros: Ros, ns: String = "", remappings: StringStringMap = [:]) {
        self.ros = ros
        namespace = ros.namespace
        unresolvedNamespace = ns
        if ns.starts(with: "~") {
            guard let resolved = ros.resolve(name: ns) else {
                return nil
            }
            construct(ns: resolved)
        } else {
            construct(ns: ns)
        }

        initRemappings(remappings: remappings)
    }

    /// Parent constructor.
    ///
    /// This version of the constructor takes a "parent" NodeHandle. If the passed "ns" is relative
    /// (does not start with a slash), it is equivalent to calling:
    ///
    /// NodeHandle child(parent.getNamespace() + "/" + ns, remappings);
    /// If the passed "ns" is absolute (does start with a slash), it is equivalent to calling:
    /// NodeHandle child(ns, remappings);
    /// This version also lets you pass in name remappings that are specific to this NodeHandle
    /// When a NodeHandle is copied, it inherits the namespace of the NodeHandle being copied, and
    /// increments the reference count of the global node state by 1.
    ///
    /// - Parameters:
    ///     - ns:    Namespace for this NodeHandle. This acts in addition to any namespace assigned to
    /// this ROS node. eg. If the node's namespace is "/a" and the namespace passed in here is "b",
    /// all topics/services/parameters will be prefixed with "/a/b/"
    ///     - remappings:    Remappings for this NodeHandle.

    internal init(parent: NodeHandle, ns: String = "", remappings: StringStringMap = [:]) {
        self.ros = parent.ros
        self.namespace = parent.namespace
        self.remappings = parent.remappings
        self.callbackQueue = parent.callbackQueue
        unresolvedNamespace = parent.unresolvedNamespace
        unresolvedRemappings = parent.unresolvedRemappings
        construct(ns: ns)
        initRemappings(remappings: remappings)
    }

    deinit {
        destruct()
    }

    private func construct(ns: String) {
        namespace = resolveName(name: ns, remap: true) ?? ""

        if ros.nodeReferenceCount.add(1) == 0 && !ros.isStarted {
            gNodeStartedByNodeHandle = true
            ros.start()
        }
    }


    // MARK: public api


    /// Advertise a topic, simple version.
    ///
    /// This call connects to the master to publicize that the node will be publishing messages on the
    /// given topic. This method returns a Publisher that allows you to publish a message on this topic.
    ///
    /// This version of advertise is a templated convenience function, and can be used like so
    ///
    /// `let pub = handle.advertise(topic: "my_topic", message: std_msgs.Empty.self)`
    ///
    /// - Parameters:
    ///     - topic:    Topic to advertise on
    ///     - queue_size:    Maximum number of outgoing messages to be queued for delivery to subscribers
    ///     - latch:  If true, the last message published on this topic will be saved and sent to new
    /// subscribers when they connect
    ///     - message: The type of the message
    /// - Returns: On success, a Publisher that, when it goes out of scope, will automatically release
    /// a reference on this advertisement.


    public func advertise<M: Message>(topic: String, latch: Bool = false, message: M.Type) -> Publisher? {
        let ops = AdvertiseOptions(topic: topic, latch: latch, M.self)
        return advertise(ops: ops)
    }

    /// Advertise a topic, with full range of AdvertiseOptions.
    ///
    /// This call connects to the master to publicize that the node will be publishing messages on the given
    //7 topic. This method returns a Publisher that allows you to publish a message on this topic.
    ///
    /// - Parameters:
    ///     - ops:    Advertise options to use
    /// - Returns: On success, a Publisher that, when it goes out of scope, will automatically release
    /// a reference on this advertisement.


    public func advertise<M: Message>(ops: AdvertiseOptions<M>) -> Publisher? {
        guard let topic = resolveName(name: ops.topic) else {
            return nil
        }

        var options = ops
        options.topic = topic
        if options.callbackQueue == nil {
            options.callbackQueue = getCallbackQueue()
        }
        let callbacks = SubscriberCallbacks(connect: options.connectCallBack,
                                            disconnect: options.disconnectCallBack,
                                            hasTrackedObject: options.trackedObject != nil,
                                            trackedObject: options.trackedObject)

        if ros.topicManager.advertise(ops: options, callbacks: callbacks) {
            let pub = SpecializedPublisher(topicManager: ros.topicManager, topic: options.topic, message: M.self, callbacks: callbacks)

            return pub
        }

        return nil
    }

    /// Advertise a service.
    ///
    /// This call connects to the master to publicize that the node will be offering
    /// an RPC service with the given name.
    ///
    /// # Usage
    ///
    ///     let srv2 = n.advertise(service: "echo") { (req: TestStringString.Request) -> TestStringString.Response? in
    ///         let response = req.data.uppercased()
    ///         return .init(response)
    ///     }
    ///
    /// - Parameters:
    ///     - service: name of service
    ///     - srvFunc: Completion to be called when service is called
    ///


    public func advertise<MReq: ServiceMessage, MRes: ServiceMessage>(
        service: String,
        srvFunc: @escaping (MReq) -> MRes?) -> ServiceServer? {

        let ops = AdvertiseServiceOptions(service: service, callback: srvFunc)
        return advertiseService(ops: ops)
    }

    /// Advertise a service with full range of *AdvertiseServiceOptions*.
    ///
    /// This call connects to the master to publicize that the node will be offering an RPC service with the
    /// given name.
    ///
    /// This version of advertiseService allows the full set of options, exposed through the
    /// *AdvertiseServiceOptions* class

    ///
    /// - Parameters:
    ///     - ops: Advertise options
    ///


    func advertiseService<MReq: ServiceMessage, MRes: ServiceMessage>(ops: AdvertiseServiceOptions<MReq, MRes>) -> ServiceServer? {
        let options = ops
        guard let service = resolveName(name: ops.service) else {
            return nil
        }
        options.service = service

        if options.callbackQueue == nil {
            options.callbackQueue = getCallbackQueue()
        }
        if ros.serviceManager.advertiseService(options) {
            let service = ServiceServer(service: options.service, node: self)
            return service
        }
        return nil
    }

    /// Create a timer which will call a callback at the specified rate, using wall time to determine when to
    /// call the callback instead of ROS time.
    ///
    /// When the Timer (and all copies of it) returned goes out of scope, the timer will automatically be
    /// stopped, and the callback will no longer be called.
    ///
    /// - Parameters:
    ///     - period: The period at which to call the callback
    ///     - oneshot: If true, this timer will only fire once
    ///     - autostart: If true (default), return timer that is already started


    func createSteadyTimer(period: WallDuration,
                           oneshot: Bool = false,
                           autostart: Bool = true,
                           trackedObject: AnyObject? = nil,
                           callback: @escaping (SteadyTimerEvent) -> Void) -> SteadyTimer  {

        let timer = SteadyTimer(period: period,
                                callback: callback,
                                callbackQueue: getCallbackQueue(),
                                trackedObject: trackedObject,
                                oneshot: oneshot)

        if autostart {
            timer.start()
        }
        return timer
    }

    /// Create a timer which will call a callback at the specified rate,
    ///
    /// When the Timer (and all copies of it) returned goes out of scope, the timer will automatically be
    /// stopped, and the callback will no longer be called.
    ///
    /// - Parameters:
    ///     - period: The period at which to call the callback
    ///     - oneshot: If true, this timer will only fire once
    ///     - autostart: If true (default), return timer that is already started


    func createTimer(period: Duration,
                     oneshot: Bool = false,
                     autostart: Bool = true,
                     trackedObject: AnyObject? = nil,
                     callback: @escaping TimerCallback) -> Timer  {

        let timer = Timer(period: period,
                          callback: callback,
                          callbackQueue: getCallbackQueue(),
                          trackedObject: trackedObject,
                          oneshot: oneshot)

        if autostart {
            timer.start()
        }
        return timer

    }

    /// Create a timer which will call a callback at the specified rate, using wall time to determine when to
    /// call the callback instead of ROS time.
    ///
    /// When the Timer (and all copies of it) returned goes out of scope, the timer will automatically be
    /// stopped, and the callback will no longer be called.
    ///
    /// - Parameters:
    ///     - period: The period at which to call the callback
    ///     - oneshot: If true, this timer will only fire once
    ///     - autostart: If true (default), return timer that is already started


    func createWallTimer(period: WallDuration,
                         oneshot: Bool = false,
                         autostart: Bool = true,
                         trackedObject: AnyObject? = nil,
                         callback: @escaping (WallTimerEvent) -> Void) -> WallTimer  {

        let timer = WallTimer(period: period,
                              callback: callback,
                              callbackQueue: getCallbackQueue(),
                              trackedObject: trackedObject,
                              oneshot: oneshot)

        if autostart {
            timer.start()
        }
        return timer
    }


    /// Delete a parameter from the parameter server.
    ///
    /// - Parameters:
    ///     - parameter: The parameter to delete

    func delete(paramter: String) -> Bool {
        guard let name = resolveName(name: paramter) else {
            return false
        }

        return ros.param.del(key: name)
    }

    /// Returns the callback queue associated with this NodeHandle.
    /// If none has been explicitly set, returns the global queue

    public func getCallbackQueue() -> CallbackQueueInterface {
        return callbackQueue != nil ? callbackQueue! : ros.getGlobalCallbackQueue()
    }


    /// Get a parameter value
    ///
    /// - Parameters:
    ///     - parameter: The key to be used in the parameter server's dictionary
    ///     - value: Storage for the retrieved value
    ///
    /// - Returns: `true` if the parameter value was retrieved, `false` otherwise

    func get<T>(parameter: String, value: inout T) -> Bool {
        return ros.param.get(parameter, &value)
    }

    /// Get a parameter value from the parameter server with local cahcing.
    ///
    /// If you want to provide a default value in case the key does not exist use param().
    ///
    /// This method will cache parameters locally, and subscribe for updates
    /// from the parameter server. Once the parameter is retrieved for the first
    /// time no subsequent getCached() calls with the same key will query
    /// the master – they will instead look up in the local cache.
    ///
    /// - Parameters:
    ///     - parameter: The key to be used in the parameter server's dictionary
    ///     - value: Storage for the retrieved value
    ///
    /// - Returns: `true` if the parameter value was retrieved, `false` otherwise

    func getCached<T>(parameter: String, value: inout T) -> Bool {
        return ros.param.getCached(parameter, &value)
    }

    /// Check whether a parameter exists on the parameter server.
    ///
    /// - Parameters:
    ///     - paramter: The parameter to check
    ///
    /// - Returns: `true` if the parameter exists, `false` otherwise

    func has(parameter: String) -> Bool {
        guard let name = resolveName(name: parameter) else {
            return false
        }


        return ros.param.has(key: name)
    }

    /// Return value from parameter server, or default if unavailable.
    ///
    /// This method tries to retrieve the indicated parameter value from the parameter server.
    /// If the parameter cannot be retrieved, `defaultValue` is returned instead
    ///
    /// - Parameters:
    ///     - name: The key to be searched on the parameter server.
    ///     - value: Storage for the retrieved value
    ///     - defaultValue: Value to return if the server doesn't contain this parameter.
    ///
    /// - Returns: `true` if the parameter was retrieved from the server, `false` otherwise


    func param<T>(name: String, value: inout T, defaultValue: T) -> Bool {
        if has(parameter: name) {
            if get(parameter: name, value: &value) {
                return true
            }
        }

        value = defaultValue
        return false
    }

    /// Return value from parameter server, or default if unavailable.
    ///
    /// This method tries to retrieve the indicated parameter value from the parameter server.
    /// If the parameter cannot be retrieved, `defaultValue` is returned instead
    ///
    /// - Parameters:
    ///     - name: The key to be searched on the parameter server.
    ///     - defaultValue: Value to return if the server doesn't contain this parameter.
    ///
    /// - Returns: The parameter value retrieved from the parameter server,
    /// or `defaultValue` if unavailable


    func param<T>(name: String, defaultValue: T) -> T {
        var paramVal = defaultValue
        _ = param(name: name, value: &paramVal, defaultValue: defaultValue)
        return paramVal
    }


    /// Resolves a name into a fully-qualified name.
    ///
    /// Resolves a name into a fully qualified name, eg. "blah" => "/namespace/blah". By default also
    /// applies any matching name-remapping rules (which were usually supplied on the command line at
    /// startup) to the given name, returning the resulting remapped name.
    ///
    /// - Parameters:
    ///     - name:    Name to remap
    ///     - remap:    Whether to apply name-remapping rules
    /// - Returns: Resolved name or nil If the name begins with a tilde '~',
    /// or is an otherwise invalid graph resource name

    func resolveName(name: String, remap: Bool = true) -> String? {
        var error = ""
        guard Names.validate(name: name, error: &error) else {
            ROS_ERROR(error)
            return nil
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
            ROS_ERROR(error)
            return nil
        } else if final.starts(with: "/") {
            // do nothing
        } else if !namespace.isEmpty {
            final = Names.append(namespace, final)
        }

        final = Names.clean(final)

        if remap {
            return remapName(name: final)
        }

        return ros.resolve(name: final, remap: false)

    }

    /// Search up the tree for a parameter with a given key.
    ///
    /// This function parameter server's searchParam feature to search up the tree for a parameter.
    /// For example, if the parameter server has a parameter [/a/b] and you're in the
    /// namespace [/a/c/d], searching for the parameter "b" will yield [/a/b].
    /// If [/a/c/d/b] existed, that parameter would be returned instead.
    ///
    /// - Parameters:
    ///     - parameter: the parameter to search for
    ///     - result: the found value (if any)
    ///
    /// - Returns: `true` if the parameter was found, `false` otherwise


    func search(parameter: String, result: inout String) -> Bool {
        // searchParam needs a separate form of remapping -- remapping on the unresolved name, rather than the
        // resolved one.

        var remapped = parameter
        if let it = unresolvedRemappings[parameter] {
            remapped = it
        }

        guard let namespace = resolveName(name: "") else {
            return false
        }

        return ros.param.search(ns: namespace, key: remapped, result: &result)
    }

    /// Create a client for a service
    ///
    /// When the last handle reference of a persistent connection is cleared,
    /// the connection will automatically close.
    ///
    /// - Parameters:
    ///     - service: The name of the service to connect to
    ///     - md5sum: checksum for the service
    ///     - persistent: Whether this connection should persist. Persistent services
    ///         keep the connection to the remote host active so that subsequent
    ///         calls will happen faster. In general persistent services are discouraged,
    ///         as they are not as robust to node failure as non-persistent services.
    ///     - headerValues: Key/value pairs you'd like to send along in the connection handshake
    ///
    ///

    func serviceClient(service: String,
                       md5sum: String,
                       persistent: Bool = false,
                       headerValues: StringStringMap? = nil) -> ServiceClient {

        /// FIXME: Find a better way to deal with wrong names
        let name = resolveName(name: service) ?? service
        let client = ServiceClient(ros: ros, name: name, md5sum: md5sum, persistent: persistent, headerValues: headerValues)

        if !client.isValid() {
            ROS_ERROR("invalid client")
        }

        return client
    }

    /// Set the default callback queue to be used by this NodeHandle.
    ///
    /// Setting this will cause any callbacks from advertisements/subscriptions/services/etc.
    /// to happen through the use of the specified queue. `nil` (the default) causes
    /// the global queue (serviced by Ros.spin() and Ros.spinOnce()) to be used.

    func setCallbackQueue(queue: CallbackQueueInterface) {
        callbackQueue = queue
    }

    /// Set a value on the parameter server.
    ///
    /// - Parameters:
    ///     - parameter: The key to be used in the parameter server's
    ///     - value: The value to be inserted.

    func set<T>(parameter: String, value: T) {
        ros.param.set(key: parameter, value: value)
    }


    /// Shutdown every handle created through this NodeHandle.
    ///
    /// This method will unadvertise every topic and service advertisement,
    /// and unsubscribe every subscription created through this NodeHandle.
    ///

    func shutdown() {
        ok = false
    }


    /// Subscribe to a topic.
    ///
    /// This method connects to the master to register interest in a given topic.
    /// The node will automatically be connected with publishers on this topic.
    /// On each message receipt, the callback is invoked and passed the received message embedded
    /// in a MessageEvent.
    ///
    /// - Parameters:
    ///     - topic: Topic to subscribe to
    ///     - queueSize: Number of incoming messages to queue up for
    ///             processing (messages in excess of this queue capacity will be
    ///             discarded).
    ///     - callback: closure to be called
    ///
    /// - Returns: On success, a Subscriber that, when all copies of it go out of scope,
    /// will unsubscribe from this topic.


    public func subscribe<M: Message>(topic: String,
                                      queueSize: UInt32,
                                      callback: @escaping (MessageEvent<M>) -> Void) -> Subscriber? {
        let ops = SubscribeOptions(topic: topic, queueSize: queueSize, queue: ros.gGlobalQueue, callback: callback)
        return subscribeWith(options: ops)
    }

    /// Subscribe to a topic.
    ///
    /// This method connects to the master to register interest in a given topic.
    /// The node will automatically be connected with publishers on this topic.
    /// On each message receipt, the callback is invoked and passed the message received.
    ///
    /// - Parameters:
    ///     - topic: Topic to subscribe to
    ///     - queueSize: Number of incoming messages to queue up for
    ///             processing (messages in excess of this queue capacity will be
    ///             discarded).
    ///     - callback: closure to be called
    ///
    /// - Returns: On success, a Subscriber that, when all copies of it go out of scope,
    /// will unsubscribe from this topic.


    public func subscribe<M: Message>(topic: String,
                                      queueSize: UInt32 = 1,
                                      callback: @escaping (M) -> Void) -> Subscriber? {
        let ops = SubscribeOptions(topic: topic, queueSize: queueSize,  queue: ros.gGlobalQueue, callback: callback)
        return subscribeWith(options: ops)
    }

    public func spinThread() {
        ros.spin()
    }


    // MARK: private functions

    private func destruct() {
        if ros.nodeReferenceCount.sub(1) == 1 && gNodeStartedByNodeHandle {
            ros.shutdown()
        }
    }

    private func initRemappings(remappings: StringStringMap) {
        remappings.forEach {
            if let resolvedKey = resolveName(name: $0.key, remap: false ),
                let resolvedName = resolveName(name: $0.value, remap: false) {
                self.remappings[resolvedKey] = resolvedName
                unresolvedRemappings[$0.key] = $0.value
            } else {
                ROS_ERROR("remapping \($0.key) to \($0.value) failed")
            }
        }
    }

    private func remapName(name: String) -> String? {
        guard let resolved = resolveName(name: name, remap: false) else {
            return nil
        }

        // First search any remappings that were passed in specifically for this NodeHandle
        if let it = remappings[resolved] {
            // ROSCPP_LOG_DEBUG("found 'local' remapping: %s", it->second.c_str());
            return it
        }

        // If not in our local remappings, perhaps in the global ones
        return ros.remap(name: resolved)
    }



    private func subscribeWith<M: Message>(options: SubscribeOptions<M>) -> Subscriber? {
        var options = options
        guard let topic = resolveName(name: options.topic) else {
            return nil
        }
        options.topic = topic

        if ros.topicManager.subscribeWith(options: options) {
            let sub = Subscriber(topic: options.topic, node: self, helper: options.helper)
            return sub
        }

        return nil
    }

}
