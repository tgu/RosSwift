import Foundation
import NIO
import Atomics
@_exported import RosTime
@_exported import StdMsgs
@_exported import msgs
import rpcobject
@_exported import RosNetwork
import Atomics
import Synchronization

public typealias StringStringMap = [String: String]

/// A unique identifier for a `Ros` instance, used to look it up in the
/// process-global registry (so detached components can find their owning node).
public struct RosID: Hashable, Sendable {
    public let id = UUID()
}

enum RosError: Error, CustomStringConvertible {
    case invalidNodeName(name: String, str: String)
    
    var description: String {
        switch self {
        case .invalidNodeName(let name, let str): "\(name ), node names cannot contain \(str)"
        }
    }
}

/// A ROS node — the central entry point of the client library.
///
/// Construct a `Ros` with a node name (and optionally command-line args /
/// remappings), then create `NodeHandle`s from it to advertise topics and
/// services, subscribe, and access parameters via ``param``. The node connects
/// to the master determined from `ROS_MASTER_URI` (or a `__master` remapping).
///
/// `Ros` conforms to `ServiceLifecycle.Service`, so it can be run inside a
/// `ServiceGroup` for structured startup/shutdown; alternatively the first
/// `NodeHandle` created starts it lazily.
///
/// ```swift
/// let ros = try Ros(name: "my_node")
/// let node = await ros.createNode()
/// let pub = await node.advertise(topic: "/chatter", message: std_msgs.string.self)
/// ```
public final class Ros: Sendable {
    /// Stable identity of this node, used by detached components to find it.
    public let id = RosID()

    /// Options controlling node initialization, passed to `Ros.init`.
    public enum InitOptions: Sendable {
        /// Don't install the library's SIGINT/SIGTERM handlers (e.g. when a
        /// `ServiceGroup` owns the signal traps).
        case noSigintHandler
        /// Append a unique suffix to the node name so multiple instances can
        /// coexist.
        case anonymousName
        /// Don't advertise/log to `/rosout`.
        case noRosout
    }
    
    fileprivate static let globalRos = Mutex<[RosID: Ros]>([:])

    static func getGlobalRos(for id: RosID) -> Ros? {
        return globalRos.withLock { $0[id] }
    }

    static func getRunningRos() -> Ros? {
        return globalRos.withLock { dict in
            if let (_, ros) = dict.first, dict.count == 1 {
                return ros
            }
            return nil
        }
    }
    
    fileprivate static let atexitRegistered = ManagedAtomic(false)
    
    /// A set of `InitOptions`.
    public typealias InitOption = Set<InitOptions>
    
    let gGlobalQueue = AsyncCallbackQueue()
    let gInternalCallbackQueue = AsyncCallbackQueue()
    let initOptions: InitOption
    
    let rosoutAppender: ROSOutAppender?
    let fileLog: FileLog
    let isShuttingDown = ManagedAtomic<Bool>(false)
    /// Whether the node is currently running (set false on shutdown). See ``ok``.
    public let isRunning = ManagedAtomic<Bool>(false)
    let _isStarted = ManagedAtomic<Bool>(false)
    var isStarted: Bool { _isStarted.load(ordering: .relaxed) }
    /// This node's interface to the ROS parameter server.
    public let param: Param
    let serviceManager: ServiceManager
    let topicManager: TopicManager
    let connectionManager: ConnectionManager
    let xmlrpcManager: XMLRPCManager
    let master: Master
    
    let network: RosNetwork
    /// The fully-resolved name of this node (including namespace and any
    /// anonymous suffix).
    public let name: String
    let namespace: String
    
    // has currently no function
    let useKeepAlive = ManagedAtomic(true)
    
    internal let nodeReferenceCount = ManagedAtomic<UInt>(0)
    internal let globalRemappings: StringStringMap
    internal let globalUnresolvedRemappings: StringStringMap
    
    let unhandledArgs: [String]
    
    /// `true` while the node is running; flips to `false` on shutdown. Use it
    /// as the condition of publish/spin loops.
    public var ok: Bool { return isRunning.load(ordering: .relaxed) }
    
    /// Alternate ROS initialization function.
    ///
    /// - Parameter remappings: A map<string, string> where each one constitutes
    /// a name remapping, or one of the special remappings like __name, __master, __ns, etc.
    /// - Parameter name: Name of this node.  The name must be a base name, ie. it cannot contain namespaces.
    /// - Parameter options: [optional] Options to start the node with (a set of bit flags from \ref ros::init_options)
    
    public init(name inName: String,
                namespace: String = "",
                remappings: StringStringMap = [:],
                unhandledArgs: [String] = [],
                options: InitOption = []) throws
    {
        if inName.isEmpty {
            fatalError("The node name must not be empty")
        }

        self.unhandledArgs = unhandledArgs
        initOptions = options
        isRunning.store(true, ordering: .relaxed)
        check_ipv6_environment()
        network = RosNetwork(remappings: remappings)
        
        let (masterHost, masterPort) = Master.determineRosMasterAddress(remappings: remappings)
        
        
        master = Master(group: threadGroup, host: masterHost, port: masterPort)
        
        var ns = ProcessInfo.processInfo.environment["ROS_NAMESPACE"] ?? namespace
        
        var node_name = inName
        
        var disableAnon = false
        if let it = remappings["__name"] {
            node_name = it
            disableAnon = true
        }
        
        if let it = remappings["__ns"] {
            ns = it
        }
        
        ns = Names.clean(ns)
        if ns.isEmpty || ns.first != "/" {
            ns = "/" + ns
        }
        
        var error = ""
        if !Names.validate(name: ns, error: &error) {
            fatalError("Namespace [\(ns)] is invalid: \(error)")
        }
        
        // names must be initialized here, because it requires the namespace
        // to already be known so that it can properly resolve names.
        // It must be done before we resolve g_name, because otherwise the name will not get remapped.
        var globalRemappings = StringStringMap()
        var globalUnresolvedRemappings = StringStringMap()
        for (key, value) in remappings {
            if !key.isEmpty && key.first! != "_" && key != node_name {
                if let resolvedKey = Names.resolve(ns: ns, name: key),
                   let resolvedName = Names.resolve(ns: ns, name: value) {
                    globalRemappings[resolvedKey] = resolvedName
                    globalUnresolvedRemappings[key] = value
                } else {
                    ROS_ERROR("remapping \(key) to \(value) failed")
                }
            }
        }
        self.globalRemappings = globalRemappings
        self.globalUnresolvedRemappings = globalUnresolvedRemappings
        
        if node_name.contains("/") {
            throw RosError.invalidNodeName(name: node_name, str: "/")
        }
        
        if node_name.contains("~") {
            throw RosError.invalidNodeName(name: node_name, str: "~")
        }
        
        node_name = Names.resolve(ns: ns, name: node_name)!
        
        if options.contains(.anonymousName) && !disableAnon {
            node_name.append("_\(ProcessInfo.processInfo.processIdentifier)")
        }
        
        Console.setFixedFilterToken(key: "node", val: node_name)
        
        self.namespace = ns
        self.name = node_name
        
        xmlrpcManager = XMLRPCManager(host: network.gHost)
        
        serviceManager = ServiceManager(rosID: id)
        topicManager = TopicManager(rosID: id, rosName: node_name)
        connectionManager = ConnectionManager(rosID: id)
        param = Param(rosID: id)
        
        fileLog = FileLog(thisNodeName: name, remappings: remappings)

        if !options.contains(.noRosout) {
            let appender = ROSOutAppender()
            Console.registerAppender(appender: appender)
            rosoutAppender = appender
        } else {
            rosoutAppender = nil
        }

        if !Ros.atexitRegistered.load(ordering: .relaxed) {
            Ros.atexitRegistered.store(true, ordering: .relaxed)
            atexit(atexitCallback)
        }
        
        Ros.globalRos.withLock { $0[self.id] = self }
        
        Task { await param.initialize(remappings: remappings) }
        
        ROS_INFO("Ros is initializing")
        
        
    }
    
    ///  ROS initialization function.
    ///
    /// This function will parse any ROS arguments (e.g., topic name
    /// remappings), and will consume them (i.e., argc and argv may be
    /// modified as a result of this call).
    ///
    /// Use this version if you are using the NodeHandle API
    ///
    /// - Parameter argv: Command line argumets
    /// - Parameter name: Name of this node.  The name must be a base name, ie.
    ///             it cannot contain namespaces.
    /// - Parameter options: [optional] Options to start the node with
    /// (a set of bit flags from `Ros.InitOption`)
    
    
    public convenience init(argv: [String], name: String, options: InitOption = []) throws {
        
        
        var remappings = StringStringMap()
        var unhandled = [String]()
        
        for arg in argv {
            if let pos = arg.range(of: ":=") {
                let local = String(arg.prefix(upTo: pos.lowerBound))
                let external = String(arg.suffix(from: pos.upperBound))
                ROS_DEBUG("remap \(local) => \(external)")
                remappings[local] = external
            } else {
                unhandled.append(arg)
            }
        }
        try self.init(name: name, remappings: remappings, unhandledArgs: unhandled, options: options)
        
        
    }
    
    /// Mainly for testing purpose
    convenience init(name: String = #function, master: String, port: Int = 11311) throws {
        var name = name
        if let firstpar = name.firstIndex(of: "(") {
            name = String(name[name.startIndex..<firstpar])
        }
        try self.init(name: name, remappings: ["__master" : "http://\(master):\(port)"], unhandledArgs: [])
    }
    
    deinit {
        shutdown()
        Ros.globalRos.withLock { _ = $0.removeValue(forKey: self.id) }
    }
    
    /// Creates a `NodeHandle` in this node's root namespace.
    ///
    /// Creating the first handle starts the node (binds its servers and
    /// connects to the master).
    public func createNode() async -> NodeHandle {
        return await NodeHandle(ros: self)!
    }
    
    
    /// Constructor.
    ///
    /// When a NodeHandle is constructed, it checks to see if the global node state has already been
    /// started. If so, it increments a global reference count. If not, it starts the node with
    /// `Ros.start()` and sets the reference count to 1.
    ///
    /// - Parameters:
    ///     - ns:    Namespace for this NodeHandle. This acts in addition to any namespace assigned to
    /// this ROS node. eg.
    /// If the node's namespace is "/a" and the namespace passed in here is "b",
    /// all topics/services/parameters will be prefixed with "/a/b/"
    ///     - remappings:    Remappings for this NodeHandle.
    
    public func createNode(ns: String, remappings: StringStringMap = [:]) async -> NodeHandle? {
        return await NodeHandle(ros: self, ns: ns, remappings: remappings)
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
    ///     - parent: The parent of the new node
    ///     - ns:    Namespace for this NodeHandle. This acts in addition to any namespace assigned to this ROS node.
    /// eg. If the node's namespace is "/a" and the namespace passed in here is "b",
    /// all topics/services/parameters will be prefixed with "/a/b/"
    
    
    public func createNode(parent: NodeHandle, ns: String = "") async -> NodeHandle {
        return await NodeHandle(parent: parent, ns: ns)
    }
    
    /// Get the list of all topics that are being published by all nodes
    ///
    /// This method communicates with the master to retrieve the list of all currently advertised topics
    /// ```swift
    ///      let ros = Ros(name: "myRos")
    ///      let topics = try await ros.getTopics()
    /// ```
    ///
    /// - Returns: a  list of topics
    
    public func getTopics() async throws -> [TopicInfo] {
        return try await master.getTopics(callerId: name)
    }
    
    
    /// Retreives the currently-known list of nodes from the master
    /// - Returns: list of nodes
    public func getNodes() async throws -> [String] {
        return try await master.getNodes(callerId: name)
    }
    
    
    /// The node's default callback queue, on which subscription and timer
    /// callbacks are delivered when no per-handle queue is specified.
    public func getGlobalCallbackQueue() -> AsyncCallbackQueue {
        return gGlobalQueue
    }
    
    func requestShutdown() {
        shutdown()
    }
    
    func shutdownCallback(params: XmlRpcValue) -> XmlRpcValue {
        var count = 0
        switch params {
        case  .array(let a):
            count = a.count
        default:
            break
        }
        if count > 1 {
            let reason = params[1]
            ROS_INFO("Shutdown request received.")
            ROS_INFO("Reason given for shutdown: \(reason)")
            // we have to avoid calling wait inside an EventLoop
            Task {
                self.requestShutdown()
            }
        }
        
        return XmlRpc.responseInt(code: 1, msg: "", response: 0)
    }
    
    
    
    func removeROSArgs(argv: [String]) -> [String] {
        return argv.filter { $0.contains(":=") }
    }
    
    /// Suspends until the node shuts down (polls ``ok``).
    public func waitForShutdown() async {
        while isRunning.load(ordering: .relaxed) {
            _ = await WallDuration(seconds: 0.05).sleep()
        }
    }
    
    internal func start() async {
        guard _isStarted.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged else {
            return
        }
        
        ROS_INFO("starting Ros")
        
        isRunning.store(true, ordering: .relaxed)
        
        let _useKeepAlive = await param.param(name: "/tcp_keepalive", defaultValue: useKeepAlive.load(ordering: .relaxed))
        useKeepAlive.store(_useKeepAlive, ordering: .relaxed)
        
        xmlrpcManager.bind(function: "shutdown", cb: shutdownCallback)
        
        await xmlrpcManager.start()
        await topicManager.start()
        await serviceManager.start()
        await connectionManager.start()
        
        if !initOptions.contains(.noSigintHandler) {
            signal(SIGINT, basicSigintHandler)
            signal(SIGTERM, basicSigintHandler)
        }
        
        Time.initialize()

        let logServiceName = resolve(name: "~set_logger_level")!
        _ = await serviceManager.advertiseService(.init(service: logServiceName,
                                                        callback: setLoggerLevel))
        
        if isShuttingDown.load(ordering: .relaxed) {
            return
        }
        
        if let enableDebug = ProcessInfo.processInfo.environment["ROSCPP_ENABLE_DEBUG"],
           enableDebug.lowercased() == "true" || enableDebug == "1" {
            
            let closeServiceName = resolve(name: "~debug/close_all_connections")!
            let options = AdvertiseServiceOptions(service: closeServiceName, callback: closeAllConnections)
            _ = await serviceManager.advertiseService(options)
        }
        
        let useSimTime = await param.param(name: "/use_sim_time", defaultValue: false)
        if useSimTime {
            Time.setNow(Time())
            let ops = SubscribeOptions(topic: "/clock", queueSize: 1, queue: getGlobalCallbackQueue(), callback: clockCallback)
            if !(await topicManager.subscribeWith(options: ops)) {
                ROS_ERROR("could not subscribe to /clock")
            }
        }
        
        if isShuttingDown.load(ordering: .relaxed) {
            return
        }
        
        ROS_INFO("Started node [\(name)], " +
                 "pid [\(getpid())], bound on [\(network.gHost)], " +
                 "xmlrpc port [\(xmlrpcManager.serverPort)], " +
                 "tcpros port [\(connectionManager.port)], using [\(Time.isSimTime ? "sim":"real")] time")
        
    }
    
    
    
    func closeAllConnections(x: std_srvs.Empty.Request) -> std_srvs.Empty.Response? {
        ROS_INFO("close_all_connections service called, closing connections")
        connectionManager.clear(reason: .transportDisconnect)
        return std_srvs.Empty.Response()
    }
    
    
    func clockCallback(msg: rosgraph_msgs.Clock) {
        Time.setNow(msg.clock)
    }
    
    /// Shuts the node down synchronously: tears down topics, services,
    /// connections and the XML-RPC server, and flips ``ok`` to false.
    ///
    /// The master `unregister*` calls are fired but not awaited; for a clean
    /// teardown that waits for them, use ``shutdownAsync()``. Idempotent.
    public func shutdown() {
        if isShuttingDown.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged {
            ROS_DEBUG("ros shutdown")
            if _isStarted.load(ordering: .relaxed) {
                topicManager.shutdown()
                serviceManager.shutdown()
                connectionManager.shutdown()
                xmlrpcManager.shutdown()
            }
            _isStarted.store(false, ordering: .relaxed)
            isRunning.store(false, ordering: .relaxed)
            isShuttingDown.store(false, ordering: .relaxed)
        }
    }

    /// Async variant of `shutdown()` that awaits the in-flight master
    /// `unregister*` calls in `TopicManager` and `ServiceManager` before
    /// returning. Use this from async contexts (tests, structured shutdown)
    /// so the master receives a clean teardown before the process exits or
    /// the master is stopped.
    public func shutdownAsync() async {
        if isShuttingDown.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged {
            ROS_DEBUG("ros shutdown (async)")
            if _isStarted.load(ordering: .relaxed) {
                await topicManager.shutdownAsync()
                await serviceManager.shutdownAsync()
                await connectionManager.shutdownAsync()
                await xmlrpcManager.shutdownAsync()
            }
            _isStarted.store(false, ordering: .relaxed)
            isRunning.store(false, ordering: .relaxed)
            isShuttingDown.store(false, ordering: .relaxed)
        }
    }
    
    /// Processes the global callback queue until the node shuts down.
    ///
    /// Runs a single-threaded spinner; call it from a task dedicated to
    /// callback delivery. Returns when ``ok`` becomes false.
    public func spin() async {
        let spinner = SingleThreadSpinner()
        await spin(spinner)
    }
    
    func spin(_ spinner: Spinner) async {
        await spinner.spin(ros: self, queue: nil)
    }
    
    /// Processes all callbacks currently ready on the global callback queue,
    /// then returns. Use in a custom loop instead of ``spin()``.
    public func spinOnce() async {
        do {
            try await gGlobalQueue.callAvailable()
        } catch let error {
            ROS_ERROR(error.localizedDescription)
        }
    }
}


private func basicSigintHandler(signal: Int32) {
    ROS_INFO("SIGINT")
    let snapshot = Ros.globalRos.withLock { Array($0.values) }
    for ros in snapshot {
        ros.requestShutdown()
    }
}

private func atexitCallback() {
    let snapshot = Ros.globalRos.withLock { Array($0.values) }
    for ros in snapshot {
        if ros.isRunning.load(ordering: .relaxed) && !ros.isShuttingDown.load(ordering: .relaxed) {
            ROS_DEBUG("shutting down due to exit() or end of main() without cleanup of all NodeHandles")
            ros.shutdown()
        }
    }
}

private func check_ipv6_environment() {
    if let envIPv6 = ProcessInfo.processInfo.environment["ROS_IPV6"] {
        let useIPv6 = envIPv6 == "on"
        if useIPv6 {
            ROS_DEBUG("ROS_IPV6 is ignored")
        }
    }
}

func amIBeingDebugged() -> Bool {
#if os(OSX)
    var info = kinfo_proc()
    var mib: [Int32] = [CTL_KERN, KERN_PROC, KERN_PROC_PID, getpid()]
    var size = MemoryLayout<kinfo_proc>.stride
    let junk = sysctl(&mib, UInt32(mib.count), &info, &size, nil, 0)
    assert(junk == 0, "sysctl failed")
    return (info.kp_proc.p_flag & P_TRACED) != 0
#else
    return false
#endif
}

