import Foundation
import HeliumLogger
import LoggerAPI
import NIO

import NIOConcurrencyHelpers
import RosTime
import StdMsgs
import rpcobject
import Network

public typealias StringStringMap = [String: String]

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

func check_ipv6_environment() {
    //        if let envIPv6 = getenv("ROS_IPV6") {
    //            let env = String(utf8String: envIPv6)
    //            let useIPv6 = env == "on"
    //        }
}


public final class Ros: Hashable {

    public static func == (lhs: Ros, rhs: Ros) -> Bool {
        return lhs === rhs
    }

    public func hash(into hasher: inout Hasher) {
        let i = ObjectIdentifier(self)
        hasher.combine(i)
    }


    public enum InitOptions {
        case noSigintHandler
        case anonymousName
        case noRosout
    }

    fileprivate static var globalRos = Set<Ros>()
    fileprivate static var atexitRegistered = false

    public typealias InitOption = Set<InitOptions>

    let gGlobalQueue = CallbackQueue()
    let gInternalCallbackQueue = CallbackQueue()
    let initOptions: InitOption

    var rosoutAppender: ROSOutAppender?
    var fileLog: FileLog?
    var isShutdownRequested = false
    var isShuttingDown = NIOAtomic.makeAtomic(value: false)
    public private(set) var isRunning = false
    var isStarted = false
    #if DEBUG
    let logg = HeliumLogger(.debug)
    #else
    let logg = HeliumLogger(.info)
    #endif
    public let param: Param
    let serviceManager: ServiceManager
    let topicManager: TopicManager
    let connectionManager: ConnectionManager
    let xmlrpcManager: XMLRPCManager
    let master: Master

    let network: Network
    public let name: String
    let namespace: String

    // has currently no function
    var useKeepAlive: Bool = true

    internal var nodeReferenceCount = NIOAtomic.makeAtomic(value: UInt(0))
    internal var globalRemappings = StringStringMap()
    internal var globalUnresolvedRemappings = StringStringMap()

    public var ok: Bool { return isRunning }

    /// Alternate ROS initialization function.
    ///
    /// - Parameter remappings: A map<string, string> where each one constitutes
    /// a name remapping, or one of the special remappings like __name, __master, __ns, etc.
    /// - Parameter name: Name of this node.  The name must be a base name, ie. it cannot contain namespaces.
    /// - Parameter options: [optional] Options to start the node with (a set of bit flags from \ref ros::init_options)

    public init(name inName: String, namespace: String = "", remappings: StringStringMap = [:], options: InitOption = []) {
        initOptions = options
        isRunning = true

        check_ipv6_environment()
        network = Network(remappings: remappings)
        master = Master(group: threadGroup)
        master.initialize(remappings: remappings)

        var ns = namespace

        if let namespaceEnvironment = ProcessInfo.processInfo.environment["ROS_NAMESPACE"] {
            ns = namespaceEnvironment
        }

        guard !inName.isEmpty else {
            fatalError("The node name must not be empty")
        }

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
        for it in remappings {
            if !it.key.isEmpty && it.key.first! != "_" && it.key != node_name {
                if let resolvedKey = Names.resolve(ns: ns, name: it.key),
                    let resolvedName = Names.resolve(ns: ns, name: it.value) {
                    globalRemappings[resolvedKey] = resolvedName
                    globalUnresolvedRemappings[it.key] = it.value
                } else {
                    ROS_ERROR("remapping \(it.key) to \(it.value) failed")
                }
            }
        }

        if node_name.contains("/") {
            fatalError("\(node_name), node names cannot contain /")
        }

        if node_name.contains("~") {
            fatalError("\(node_name), node names cannot contain ~")
        }

        node_name = Names.resolve(ns: ns, name: node_name)!

        if options.contains(.anonymousName) && !disableAnon {
            node_name.append("_\(ProcessInfo.processInfo.processIdentifier)")
        }

        Console.setFixedFilterToken(key: "node", val: node_name)

        self.namespace = ns
        self.name = node_name

        xmlrpcManager = XMLRPCManager(host: network.gHost)

        serviceManager = ServiceManager()
        topicManager = TopicManager()
        connectionManager = ConnectionManager()
        param = Param()
        param.ros = self
        param.initialize(remappings: remappings)

        fileLog = FileLog(thisNodeName: name, remappings: remappings)

        if !Ros.atexitRegistered {
            Ros.atexitRegistered = true
            atexit(atexitCallback)
        }

        Ros.globalRos.insert(self)

        Log.logger = logg
        #if os(Linux)
        logg.colored = true
        logg.details = true
        #else
        logg.colored = !amIBeingDebugged()
        logg.details = amIBeingDebugged()
        #endif
        logg.dateFormat = "HH:mm:ss.SSS"
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


    public convenience init(argv: inout [String], name: String, options: InitOption = []) {


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
        argv = unhandled
        self.init(name: name, remappings: remappings, options: options)


    }

    deinit {
        shutdown()
        Ros.globalRos.remove(self)
    }

    public func createNode() -> NodeHandle {
        return NodeHandle(ros: self)!
    }


    /// Constructor.
    ///
    /// When a NodeHandle is constructed, it checks to see if the global node state has already been
    /// started. If so, it increments a global reference count. If not, it starts the node with
    /// `Ros.start()` and sets the reference count to 1.
    ///
    /// - Parameters:
    ///     - ns:    Namespace for this NodeHandle. This acts in addition to any namespace assigned to
    /// this ROS node. eg. If the node's namespace is "/a" and the namespace passed in here is "b",
    /// all topics/services/parameters will be prefixed with "/a/b/"
    ///     - remappings:    Remappings for this NodeHandle.

    public func createNode(ns: String, remappings: StringStringMap = [:]) -> NodeHandle? {
        return NodeHandle(ros: self, ns: ns, remappings: remappings)
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
    

    public func createNode(parent: NodeHandle, ns: String = "") -> NodeHandle {
        return NodeHandle(parent: parent, ns: ns)
    }

    public func getGlobalCallbackQueue() -> CallbackQueue {
        return gGlobalQueue
    }

    func requestShutdown() {
        isShutdownRequested = true
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
            DispatchQueue(label: "shutdown").async {
                self.requestShutdown()
            }
        }

        return XmlRpc.responseInt(code: 1, msg: "", response: 0)
    }



    func removeROSArgs(argv: [String]) -> [String] {
        return argv.filter { $0.contains(":=") }
    }

    public func waitForShutdown() {
        while isRunning {
            _ = WallDuration(seconds: 0.05).sleep()
        }
    }

    private func kill() {
        ROS_ERROR("Caught kill, stopping...")
        DispatchQueue.main.async {
            self.isShutdownRequested = true
            self.requestShutdown()
        }
    }

    internal func start() {
        if isStarted {
            return
        }

        ROS_INFO("starting Ros")

        isShutdownRequested = false
        isStarted = true
        isRunning = true

        _ = param.param(name: "/tcp_keepalive", value: &useKeepAlive, defaultValue: useKeepAlive)

        xmlrpcManager.bind(function: "shutdown", cb: shutdownCallback)

        topicManager.start(ros: self)
        serviceManager.start(ros: self)
        connectionManager.start(ros: self)
        xmlrpcManager.start(host: network.gHost)

        if !initOptions.contains(.noSigintHandler) {
            signal(SIGINT, basicSigintHandler)
            signal(SIGTERM, basicSigintHandler)
        }

        Time.initialize()

        if !initOptions.contains(.noRosout) {
            let appender = ROSOutAppender()
            Console.registerAppender(appender: appender)
            rosoutAppender = appender
        }

        let logServiceName = resolve(name: "~set_logger_level")!
        _ = serviceManager.advertiseService(.init(service: logServiceName,
                                                  callback: setLoggerLevel))

        if isShuttingDown.load() {
            return
        }

        if let enableDebug = ProcessInfo.processInfo.environment["ROSCPP_ENABLE_DEBUG"],
            enableDebug.lowercased() == "true" || enableDebug == "1" {

            let closeServiceName = resolve(name: "~debug/close_all_connections")!
            let options = AdvertiseServiceOptions(service: closeServiceName, callback: closeAllConnections)
            _ = serviceManager.advertiseService(options)
        }

        let useSimTime = param.param(name: "/use_sim_time", defaultValue: false)
        if useSimTime {
            Time.setNow(Time())
        }

        if useSimTime {
            let ops = SubscribeOptions(topic: "/clock", queueSize: 1, queue: getGlobalCallbackQueue(), callback: clockCallback)
            if !topicManager.subscribeWith(options: ops) {
                ROS_ERROR("could not subscribe to /clock")
            }
        }

        if isShuttingDown.load() {
            return
        }

        ROS_INFO("Started node [\(name)], " +
            "pid [\(getpid())], bound on [\(network.gHost)], " +
            "xmlrpc port [\(xmlrpcManager.serverPort)], " +
            "tcpros port [\(connectionManager.getTCPPort())], using [\(Time.isSimTime() ? "sim":"real")] time")

    }



    func closeAllConnections(x: EmptyRequest) -> EmptyResponse? {
        ROS_INFO("close_all_connections service called, closing connections")
        connectionManager.clear(reason: .transportDisconnect)
        return EmptyResponse()
    }


    func clockCallback(msg: RosgraphMsgs.Clock) {
        Time.setNow(msg.time)
    }

    func shutdown() {


        if isShuttingDown.compareAndExchange(expected: false, desired: true) {
            ROS_DEBUG("ros shutdown")
            if isStarted {
                topicManager.shutdown()
                serviceManager.shutdown()
                connectionManager.shutdown()
                xmlrpcManager.shutdown()
            }

            isStarted = false
            isRunning = false
//            promise.succeed(())
            isShuttingDown.store(false)
        }
    }

    public func spin() {
        let spinner = SingleThreadSpinner()
        spin(spinner)
    }

    func spin(_ spinner: Spinner) {
        spinner.spin(ros: self, queue: nil)
    }

    public func spinOnce() {
        gGlobalQueue.callAvailable()
    }



}


func basicSigintHandler(signal: Int32) {
    ROS_INFO("SIGINT")
    Ros.globalRos.forEach{ $0.requestShutdown() }
}

func atexitCallback() {
    Ros.globalRos.forEach { ros in
        if ros.isRunning && !ros.isShuttingDown.load() {
            ROS_DEBUG("shutting down due to exit() or end of main() without cleanup of all NodeHandles")
            ros.shutdown()
        }
    }
}
