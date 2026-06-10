//
//  NodeHandle.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import NIOConcurrencyHelpers
import StdMsgs
import RosTime
import Atomics
import Synchronization


/// Interface for creating subscribers, publishers, etc.
///
/// This class is used for writing nodes. It provides a RAII interface to this process' node,
/// in that when the first NodeHandle is created, it instantiates everything necessary for this node,
/// and when the last NodeHandle goes out of scope it shuts down the node.
///
/// NodeHandle uses reference counting internally, and copying a NodeHandle is very lightweight.
///
/// You must call one of the `Ros.createNode(...)` functions to instantiate this class.
///
/// The most widely used methods are:
///
/// ## Setup:
///     let ros = Ros(name: "my_ros_node")
///     let node = ros.createNode()
/// ## Publish / subscribe messaging:
///     `advertise(...)`
///     `subscribe(...)`
/// ## RPC services:
///     `advertise(service:)`
///     `serviceClient(...)`
///     ros::service::call()
/// ## Parameters:
///     `get(...)`
///     `set(...)`
///

public final class NodeHandle: Sendable {

    /// Check whether it's time to exit.
    ///
    /// This method checks to see if both Ros.ok() is true and shutdown()
    /// has not been called on this NodeHandle, to see whether it's yet time to exit.
    /// `ok` is false once either Ros.shutdown() or NodeHandle.shutdown() have been called

    public var isOK: Bool { return ros.isRunning.load(ordering: .relaxed) && ok.load(ordering: .relaxed) }

    ///  the namespace associated with this NodeHandle.
    public var namespace: String { state.withLock { $0.namespace } }

    /// the namespace associated with this NodeHandle as it was passed in (before it was resolved)
    public let unresolvedNamespace: String

    /// The name remappings in effect for this handle (resolved keys → values).
    public var remappings: StringStringMap { state.withLock { $0.remappings } }

    /// The name remappings as originally passed in, before resolution.
    public var unresolvedRemappings: StringStringMap { state.withLock { $0.unresolvedRemappings } }


    /// Per-handle "is open" flag; cleared by ``shutdown()``. Combined with the
    /// node's running state in ``isOK``.
    public let ok = ManagedAtomic(true)

    /// Backing flag recording whether this handle was the one that started the node.
    public let _gNodeStartedByNodeHandle = ManagedAtomic(false)
    /// Whether this handle triggered the node's lazy start (and is therefore
    /// responsible for shutting it down when the last handle is released).
    public var gNodeStartedByNodeHandle: Bool { _gNodeStartedByNodeHandle.load(ordering: .relaxed)}

    /// The current number of live `NodeHandle`s sharing this node.
    public var refCount: UInt {
        return ros.nodeReferenceCount.load(ordering: .relaxed)
    }

    /// The node this handle belongs to.
    public unowned let ros: Ros

    /// The `host:port` of the ROS master this node is connected to.
    public var rosMasterPath: String {
        return ros.master.path
    }

    /// Mutable state that the protocol exposes via `var ... { get set }`-like
    /// public getters, plus the optional callback queue. Set during init and
    /// `initRemappings`; locked for reads to satisfy checked `Sendable`.
    private struct State: Sendable {
        var namespace: String = "/"
        var remappings = StringStringMap()
        var unresolvedRemappings = StringStringMap()
        var callbackQueue: AsyncCallbackQueue?
    }
    private let state = Mutex(State())

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
    ///     - ros:  Parent ros
    ///     - ns:    Namespace for this NodeHandle. This acts in addition to any namespace assigned to
    /// this ROS node. eg. If the node's namespace is "/a" and the namespace passed in here is "b",
    /// all topics/services/parameters will be prefixed with "/a/b/"
    ///     - remappings:    Remappings for this NodeHandle.

    internal init?(ros: Ros, ns: String = "", remappings: StringStringMap = [:]) async {
        self.ros = ros
        self.unresolvedNamespace = ns
        state.withLock { $0.namespace = ros.namespace }
        if ns.starts(with: "~") {
            guard let resolved = ros.resolve(name: ns) else {
                return nil
            }
            await construct(ns: resolved)
        } else {
            await construct(ns: ns)
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

    internal init(parent: NodeHandle, ns: String = "", remappings: StringStringMap = [:]) async {
        self.ros = parent.ros
        self.unresolvedNamespace = parent.unresolvedNamespace
        let parentSnapshot = parent.state.withLock {
            (namespace: $0.namespace,
             remappings: $0.remappings,
             unresolvedRemappings: $0.unresolvedRemappings,
             callbackQueue: $0.callbackQueue)
        }
        state.withLock {
            $0.namespace = parentSnapshot.namespace
            $0.remappings = parentSnapshot.remappings
            $0.unresolvedRemappings = parentSnapshot.unresolvedRemappings
            $0.callbackQueue = parentSnapshot.callbackQueue
        }
        await construct(ns: ns)
        initRemappings(remappings: remappings)
    }

    deinit {
        if ros.nodeReferenceCount.loadThenWrappingDecrement(ordering: .relaxed) == 1 && _gNodeStartedByNodeHandle.load(ordering: .relaxed) {
            ros.shutdown()
        }
    }

    private func construct(ns: String) async {
        let resolved = resolveName(name: ns, remap: true) ?? ""
        state.withLock { $0.namespace = resolved }

        if ros.nodeReferenceCount.loadThenWrappingIncrement(ordering: .relaxed) == 0 && !ros.isStarted {
            _gNodeStartedByNodeHandle.store(true, ordering: .relaxed)
            await ros.start()
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
    ///     let pub = node.advertise(topic: "my_topic",
    ///                              message: std_msgs.Empty.self)
    ///
    /// - Parameters:
    ///     - topic:    Topic to advertise on
    ///     - queueSize:    Maximum number of outgoing messages to be queued for delivery to subscribers
    ///     - latch:  If true, the last message published on this topic will be saved and sent to new
    /// subscribers when they connect
    ///     - message: The type of the message
    /// - Returns: On success, a Publisher that, when it goes out of scope, will automatically release
    /// a reference on this advertisement.


    public func advertise<M: Message>(topic: String, queueSize: UInt = 0, connectCall: SubscriberStatusCallback? = nil, disconnectCall: SubscriberStatusCallback? = nil, latch: Bool = false, message: M.Type, tracked_object: TrackableObject? = nil) async -> SpecializedPublisher<M>? {
        let ops = AdvertiseOptions(topic: topic, queueSize: queueSize, latch: latch, M.self, connectCall: connectCall, disconnectCall: disconnectCall, tracked_object: tracked_object)
        return await advertise(ops: ops)
    }

    /// Advertise a topic, with full range of AdvertiseOptions.
    ///
    /// This call connects to the master to publicize that the node will be publishing messages on the given
    /// topic. This method returns a Publisher that allows you to publish a message on this topic.
    ///
    /// - Parameters:
    ///     - ops:    Advertise options to use
    /// - Returns: On success, a Publisher that, when it goes out of scope, will automatically release
    /// a reference on this advertisement.


    public func advertise<M: Message>(ops: AdvertiseOptions<M>) async -> SpecializedPublisher<M>? {
        guard let topic = resolveName(name: ops.topic) else {
            return nil
        }

        var options = ops.set(topic: topic)
        if options.callbackQueue == nil {
            options = options.set(callbackQueue: getCallbackQueue())
        }
        let callbacks = SubscriberCallbacks(connect: options.connectCallBack,
                                            disconnect: options.disconnectCallBack,
                                            hasTrackedObject: options.trackedObject != nil,
                                            trackedObject: options.trackedObject)

        if await ros.topicManager.advertise(ops: options, callbacks: callbacks) {
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


    public func advertise<MReq: ServiceRequestMessage, MRes: ServiceResponseMessage>(
        service: String,
        srvFunc: @escaping @Sendable (MReq) -> MRes?,
        tracked_object: TrackableObject? = nil
    ) async -> ServiceServer? {

        let ops = AdvertiseServiceOptions(service: service, callback: srvFunc, trackedObject: tracked_object)
        return await advertiseService(ops: ops)
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


    func advertiseService<MReq: ServiceRequestMessage, MRes: ServiceResponseMessage>(ops: AdvertiseServiceOptions<MReq, MRes>) async -> ServiceServer? {
        var options = ops
        guard let service = resolveName(name: ops.service) else {
            return nil
        }
        options = options.set(service: service)

        if options.callbackQueue == nil {
            options = options.set(callbackQueue: getCallbackQueue())
        }
        if await ros.serviceManager.advertiseService(options) {
            return ServiceServer(service: options.service, manager: ros.serviceManager)
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
                           trackedObject: TrackableObject? = nil,
                           callback: @escaping @Sendable (SteadyTimerEvent) async -> Void) async -> SteadyTimer  {

        let timer = SteadyTimer(period: period,
                                callback: callback,
                                callbackQueue: getCallbackQueue(),
                                trackedObject: trackedObject,
                                oneshot: oneshot)

        if autostart {
            await timer.start()
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


    public func createTimer(period: RosDuration,
                     oneshot: Bool = false,
                     autostart: Bool = true,
                     trackedObject: TrackableObject? = nil,
                     callback: @escaping TimerCallback) async -> Timer  {

        let timer = Timer(period: period,
                          callback: callback,
                          callbackQueue: getCallbackQueue(),
                          trackedObject: trackedObject,
                          oneshot: oneshot)

        if autostart {
            await timer.start()
        }
        return timer

    }

    /// Create a timer that fires at the given `Rate` (ROS time).
    ///
    /// Convenience over ``createTimer(period:oneshot:autostart:trackedObject:callback:)``
    /// using the rate's expected cycle time. The timer stops when the returned
    /// `Timer` (and all copies) go out of scope.
    ///
    /// - Parameters:
    ///     - rate: The rate at which to call the callback.
    ///     - oneshot: If true, the timer fires only once.
    ///     - autostart: If true (default), the returned timer is already started.
    public func createTimer(rate: Rate,
                     oneshot: Bool = false,
                     autostart: Bool = true,
                     trackedObject: TrackableObject? = nil,
                     callback: @escaping TimerCallback) async -> Timer  {

        await createTimer(period: rate.expectedCycleTime, oneshot: oneshot, autostart: autostart, trackedObject: trackedObject, callback: callback)

    }


    /// Create a timer that uses wall-clock time (rather than ROS time) to
    /// decide when to call the callback.
    ///
    /// The timer stops when the returned `WallTimer` (and all copies) go out of
    /// scope.
    ///
    /// - Parameters:
    ///     - period: The period at which to call the callback.
    ///     - oneshot: If true, the timer fires only once.
    ///     - autostart: If true (default), the returned timer is already started.
    public func createWallTimer(period: WallDuration,
                         oneshot: Bool = false,
                         autostart: Bool = true,
                         trackedObject: TrackableObject? = nil,
                         callback: @escaping @Sendable (WallTimerEvent) -> Void) async -> WallTimer  {

        let timer = WallTimer(period: period,
                              callback: callback,
                              callbackQueue: getCallbackQueue(),
                              trackedObject: trackedObject,
                              oneshot: oneshot)

        if autostart {
            await timer.start()
        }
        return timer
    }


    /// Delete a parameter from the parameter server.
    ///
    /// - Parameters:
    ///     - parameter: The parameter to delete

    func delete(paramter: String) async -> Bool {
        guard let name = resolveName(name: paramter) else {
            return false
        }

        return await ros.param.del(key: name)
    }

    /// Returns the callback queue associated with this NodeHandle.
    /// If none has been explicitly set, returns the global queue

    public func getCallbackQueue() -> AsyncCallbackQueue {
        return state.withLock { $0.callbackQueue } ?? ros.getGlobalCallbackQueue()
    }


    /// Get a parameter value
    ///
    /// - Parameters:
    ///     - parameter: The key to be used in the parameter server's dictionary
    ///     - value: Storage for the retrieved value
    ///
    /// - Returns: `true` if the parameter value was retrieved, `false` otherwise

    public func get<T: Sendable>(parameter: String, value: inout T) async -> Bool {
        if let resolvedName = resolveName(name: parameter) {
            return await ros.param.get(resolvedName, &value)
        }
        return false
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

    func getCached<T: Sendable>(parameter: String, value: inout T) async -> Bool {
        return await ros.param.getCached(parameter, &value) {_ in }
    }

    /// Check whether a parameter exists on the parameter server.
    ///
    /// - Parameters:
    ///     - paramter: The parameter to check
    ///
    /// - Returns: `true` if the parameter exists, `false` otherwise

    public func has(parameter: String) async -> Bool {
        guard let name = resolveName(name: parameter) else {
            return false
        }


        return await ros.param.has(key: name)
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


    public func param<T: Sendable>(name: String, value: inout T, defaultValue: T) async -> Bool {
        if await has(parameter: name) {
            if await get(parameter: name, value: &value) {
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


    func param<T: Sendable>(name: String, defaultValue: T) async -> T {
        var paramVal = defaultValue
        _ = await param(name: name, value: &paramVal, defaultValue: defaultValue)
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

        // Snapshot namespace once per call to avoid re-acquiring the state lock
        // and to keep both reads consistent.
        let snapshotNamespace = state.withLock { $0.namespace }

        if name.isEmpty {
            return snapshotNamespace
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
        } else if !snapshotNamespace.isEmpty {
            final = Names.append(snapshotNamespace, final)
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


    public func search(parameter: String, result: inout String) async -> Bool {
        // searchParam needs a separate form of remapping -- remapping on the unresolved name, rather than the
        // resolved one.

        var remapped = parameter
        if let it = unresolvedRemappings[parameter] {
            remapped = it
        }

        guard let namespace = resolveName(name: "") else {
            return false
        }

        return await ros.param.search(ns: namespace, key: remapped, result: &result)
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

    public func serviceClient(service: String,
                       md5sum: String,
                       persistent: Bool = false,
                              headerValues: StringStringMap? = nil) async -> ServiceClient {

        /// FIXME: Find a better way to deal with wrong names
        let name = resolveName(name: service) ?? service
        let client = await ServiceClient(ros: ros, name: name, md5sum: md5sum, persistent: persistent, headerValues: headerValues)

        if !client.isValid() {
            ROS_ERROR("invalid client")
        }

        return client
    }

    /// Set a value on the parameter server.
    ///
    /// - Parameters:
    ///     - parameter: The key to be used in the parameter server's
    ///     - value: The value to be inserted.

    public func set<T: Sendable>(parameter: String, value: T) async {
        if let name = resolveName(name: parameter) {
            await ros.param.set(key: name, value: value)
        }
    }


    /// Shutdown every handle created through this NodeHandle.
    ///
    /// This method will unadvertise every topic and service advertisement,
    /// and unsubscribe every subscription created through this NodeHandle.
    ///

    func shutdown() {
        ok.store(false, ordering: .relaxed)
    }


    /// Subscribe to a topic.
    ///
    /// This method connects to the master to register interest in a given topic.
    /// The node will automatically be connected with publishers on this topic.
    /// On each message receipt, the callback is invoked and passed the received message embedded
    /// in a `MessageEvent`.
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
                                      callback: @escaping @Sendable (MessageEvent<M>) -> Void) async -> Subscriber? {
        let ops = SubscribeOptions(topic: topic, queueSize: queueSize, queue: ros.gGlobalQueue, callback: callback)
        return await subscribeWith(options: ops)
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
                                      callback: @escaping @Sendable (M) -> Void,
                                      tracked_object: TrackableObject? = nil) async -> Subscriber? {
        let ops = SubscribeOptions(
            topic: topic,
            queueSize: queueSize,
            queue: ros.gGlobalQueue,
            trackedObject: tracked_object,
            callback: callback)
        return await subscribeWith(options: ops)
    }

    /// Subscribes to `topic` and delivers messages as an `AsyncStream`.
    ///
    /// An ergonomic alternative to the callback-based `subscribe`: iterate the
    /// returned stream with `for await msg in …`. The underlying subscription
    /// lives as long as the stream is being consumed; cancel the consuming task
    /// to unsubscribe.
    ///
    /// - Parameters:
    ///     - topic: The topic name to subscribe to.
    ///     - queueSize: Incoming message queue depth (default 1).
    ///     - msg: The message type to receive.
    /// - Returns: A stream of incoming messages.
    public func subscribe<M: Message>(topic: String,
                                      queueSize: UInt32 = 1,
                                      msg: M.Type
                                      ) -> AsyncStream<M> {
        AsyncStream { continuation in
            Task {
                let vab = await self.subscribe(topic: topic, queueSize: queueSize) { (msg: M) in
                    continuation.yield(msg)
                }

                extendLifetime(vab)

                while Task.isCancelled == false {
                    try? await Task.sleep(for: .seconds(1))
                }
            }
        }
    }


    /// Processes this node's callback queue until shutdown (delegates to
    /// ``Ros/spin()``). Run it from a dedicated task.
    public func spinThread() async {
        await ros.spin()
    }


    // MARK: private functions

    private func initRemappings(remappings: StringStringMap) {
        for (key, value) in remappings {
            // resolveName reads state under the lock; resolve OUTSIDE the
            // write-lock acquire below to avoid re-entrant locking.
            if let resolvedKey = resolveName(name: key, remap: false),
               let resolvedName = resolveName(name: value, remap: false) {
                state.withLock {
                    $0.remappings[resolvedKey] = resolvedName
                    $0.unresolvedRemappings[key] = value
                }
            } else {
                ROS_ERROR("remapping \(key) to \(value) failed")
            }
        }
    }

    private func remapName(name: String) -> String? {
        guard let resolved = resolveName(name: name, remap: false) else {
            return nil
        }

        // First search any remappings that were passed in specifically for this NodeHandle
        if let mapped = state.withLock({ $0.remappings[resolved] }) {
            return mapped
        }

        // If not in our local remappings, perhaps in the global ones
        return ros.remap(name: resolved)
    }



    private func subscribeWith<M: Message>(options: SubscribeOptions<M>) async -> Subscriber? {
        var options = options
        guard let topic = resolveName(name: options.topic) else {
            return nil
        }
        options.topic = topic

        if await ros.topicManager.subscribeWith(options: options) {
            let sub = Subscriber(topic: options.topic, node: self, helper: options.helper)
            return sub
        }

        return nil
    }

}
