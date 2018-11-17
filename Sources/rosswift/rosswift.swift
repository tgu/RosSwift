import Foundation
import HeliumLogger
import LoggerAPI
import NIO
import NIOConcurrencyHelpers
import RosTime
import StdMsgs

public typealias StringStringMap = [String: String]

struct TransportTCP {
    static var useKeepalive = false
    static var useIPv6 = false
}

func basicSigintHandler(signal: Int32) {
    ROS_INFO("SIGINT")
    Ros.requestShutdown()
}

func atexitCallback() {
    if Ros.isRunning && !Ros.isShuttingDown.load() {
        ROS_DEBUG("shutting down due to exit() or end of main() without cleanup of all NodeHandles")
        Ros.isStarted = false
        Ros.shutdown()
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

public struct Ros {
        public enum InitOptions {
            case noSigintHandler
            case anonymousName
            case noRosout
        }

    public typealias InitOption = Set<InitOptions>

    static var rosoutAppender: ROSOutAppender?
    static var fileLog: FileLog?
    static var initOptions = InitOption()
    static var isShutdownRequested = false
    static var isShuttingDown = Atomic<Bool>(value: false)
    static var isRunning = false
    static var isStarted = false
    static var isInitialized = false
    static var atexitRegistered = false
    static let logg = HeliumLogger(.debug)

    public static var ok: Bool { return isRunning }

    static func requestShutdown() {
        isShutdownRequested = true
        shutdown()
    }

    static func shutdownCallback(params: XmlRpcValue) -> XmlRpcValue {
        var count = 0
        switch params.getType() {
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
                requestShutdown()
            }
        }

        return XmlRpc.responseInt(code: 1, msg: "", response: 0)
    }

    /** ROS initialization function.

      This function will parse any ROS arguments (e.g., topic name
      remappings), and will consume them (i.e., argc and argv may be
      modified as a result of this call).

      Use this version if you are using the NodeHandle API

    - Parameter argv: Command line argumets
    - Parameter name: Name of this node.  The name must be a base name, ie.
                it cannot contain namespaces.
    - Parameter options: [optional] Options to start the node with
                (a set of bit flags from `Ros.InitOption`)

     */

    public static func initialize(argv: inout [String],
                                  name: String,
                                  options: InitOption = .init()) -> EventLoopFuture<Void> {

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
        return initialize(remappings: remappings, name: name, options: options)
    }

    /** Alternate ROS initialization function.

       Alternate ROS initialization function

    - Parameter remappings: A map<string, string> where each one constitutes
        a name remapping, or one of the special remappings like __name, __master, __ns, etc.
    - Parameter name: Name of this node.  The name must be a base name, ie. it cannot contain namespaces.
    - Parameter options: [optional] Options to start the node with (a set of bit flags from \ref ros::init_options)
     */

    static let promise: EventLoopPromise<Void> = threadGroup.next().newPromise()

    public static func initialize(remappings: StringStringMap,
                                  name: String,
                                  options: InitOption) -> EventLoopFuture<Void> {

        if !atexitRegistered {
            atexitRegistered = true
            atexit(atexitCallback)
        }

        initOptions = options
        isRunning = true

        check_ipv6_environment()
        Network.initialize(remappings: remappings)
        Master.shared.initialize(remappings: remappings)
        ThisNode.initialize(name: name, remappings: remappings, options: options)
        fileLog = FileLog(remappings: remappings)
        Param.initialize(remappings: remappings)

        isInitialized = true

        return Ros.promise.futureResult
    }

    static func check_ipv6_environment() {
        if let envIPv6 = getenv("ROS_IPV6") {
            let env = String(utf8String: envIPv6)
            let useIPv6 = env == "on"
        }
    }

    func removeROSArgs(argv: [String]) -> [String] {
        return argv.filter { $0.contains(":=") }
    }

    public static func waitForShutdown() {
        while isRunning {
            _ = RosTime.WallDuration(seconds: 0.05).sleep()
        }
        promise.succeed(result: Void())
    }

    private static func kill() {
        ROS_ERROR("Caught kill, stopping...")
        DispatchQueue.main.async {
            isShutdownRequested = true
            requestShutdown()
        }
    }

    static func start() {
        ROS_INFO("starting Ros")
        if isStarted {
            return
        }

        isShutdownRequested = false
        isStarted = true
        isRunning = true
        let enableDebug = false

        _ = Param.param(name: "/tcp_keepalive", value: &TransportTCP.useKeepalive, defaultValue: TransportTCP.useKeepalive)

        guard XMLRPCManager.instance.bind(function: "shutdown", cb: shutdownCallback) else {
            fatalError("Could not bind function")
        }

        initInternalTimerManager()

        TopicManager.instance.start()
        ServiceManager.instance.start()
        Ros.ConnectionManager.instance.start()
        XMLRPCManager.instance.start()

        if !initOptions.contains(.noSigintHandler) {
            signal(SIGINT, basicSigintHandler)
            signal(SIGTERM, basicSigintHandler)
        }

        RosTime.Time.initialize()

        if !initOptions.contains(.noRosout) {
            let appender = ROSOutAppender()
            Console.registerAppender(appender: appender)
            rosoutAppender = appender
        }

        _ = ServiceManager.instance.advertiseService(.init(service: "~debug/close_all_connections",
                                                           callback: closeAllConnections))
        _ = ServiceManager.instance.advertiseService(.init(service: "~set_logger_level",
                                                           callback: setLoggerLevel))

        if isShuttingDown.load() {
            return
        }

        if enableDebug {
            _ = ServiceManager.instance.advertiseService(.init(service: "~debug/close_all_connections",
                                                               callback: closeAllConnections))
        }

        var useSimTime = false
        _ = Param.param(name: "/use_sim_time", value: &useSimTime, defaultValue: useSimTime)

        if useSimTime {
            RosTime.Time.setNow(RosTime.Time())
        }

        if useSimTime {
            let ops = SubscribeOptions(topic: "/clock", callback: clockCallback)
            if !TopicManager.instance.subscribeWith(options: ops) {
                ROS_ERROR("could not subscribe to /clock")
            }
        }

        if isShuttingDown.load() {
            return
        }

        ROS_INFO("Started node [\(Ros.ThisNode.getName())], pid [\(getpid())], bound on [\(Network.getHost())], xmlrpc port [\(XMLRPCManager.instance.serverPort)], tcpros port [\(Ros.ConnectionManager.instance.getTCPPort())], using [real] time")

    }

    struct Logger: Message {

        static let md5sum = "a6069a2ff40db7bd32143dd66e1f408e"
        static let datatype = "roscpp/Logger"
        static let hasHeader = false
        static let definition = "string name\nstring level\n"

        let name: String
        let level: String

        init() {
            name = ""
            level = ""
        }

    }

    struct GetLoggersRequest: ServiceMessage {
        static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
        static let srvMd5sum = GetLoggers.md5sum
        static let srvDatatype = GetLoggers.datatype
        static let datatype = "roscpp/GetLoggersRequest"
        static let hasHeader = false
        static let definition = "\n"
    }

    struct GetLoggersResponse: ServiceMessage {
        static let md5sum = "32e97e85527d4678a8f9279894bb64b0"
        static let srvMd5sum = GetLoggers.md5sum
        static let srvDatatype = GetLoggers.datatype
        static let datatype = "roscpp/GetLoggersResponse"
        static let hasHeader = false
        static let definition = """
            Logger[] loggers

            ================================================================================\n\
            MSG: roscpp/Logger
            string name
            string level
            """

        let loggers: [Logger]

        init() {
            self.loggers = [Logger]()
        }
    }

    struct GetLoggers {
        typealias Request = GetLoggersRequest
        typealias Response = GetLoggersResponse
        static let md5sum = "32e97e85527d4678a8f9279894bb64b0"
        static let datatype = "roscpp/GetLoggers"
    }

    static func getLoggers(x: GetLoggersRequest, y: inout GetLoggersRequest) -> Bool {
        ROS_ERROR("getLoggers not implemented")
        return false
    }

    struct SetLoggerLevelRequest: ServiceMessage {
        static let md5sum = "51da076440d78ca1684d36c868df61ea"
        static let datatype = "roscpp/SetLoggerLevelRequest"
        static let hasHeader = false
        static let definition = "string logger\nstring level\n"
        static let srvMd5sum = SetLoggerLevel.md5sum
        static let srvDatatype = SetLoggerLevel.datatype

        let logger: String
        let level: String

        init() {
            logger = ""
            level = ""
        }
    }

    struct SetLoggerLevelResponse: ServiceMessage {
        static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
        static let datatype = "roscpp/SetLoggerLevelResponse"
        static let hasHeader = false
        static let definition = "\n"
        static let srvMd5sum = SetLoggerLevel.md5sum
        static let srvDatatype = SetLoggerLevel.datatype
    }

    struct SetLoggerLevel {
        typealias Request = SetLoggerLevelRequest
        typealias Response = SetLoggerLevelResponse
        static let md5sum = "51da076440d78ca1684d36c868df61ea"
        static let datatype = "roscpp/SetLoggerLevel"
    }

    static func setLoggerLevel(x: SetLoggerLevelRequest) -> SetLoggerLevelResponse? {
        ROS_ERROR("Set logger level \(x.level) for logger \(x.logger) --- Not implemented")

        return nil
    }

    struct EmptyRequest: ServiceMessage {
        static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
        static let srvMd5sum = Empty.md5sum
        static let srvDatatype = Empty.datatype
        static let datatype = "roscpp/EmptyRequest"
        static let hasHeader = false
        static let definition = "\n"
    }

    struct EmptyResponse: ServiceMessage {
        static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
        static let srvMd5sum = Empty.md5sum
        static let srvDatatype = Empty.datatype
        static let datatype = "roscpp/SetLoggerLevelResponse"
        static let hasHeader = false
        static let definition = "\n"
    }

    struct Empty {
        typealias Request = EmptyRequest
        typealias Response = EmptyResponse

        static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
        static let datatype = "roscpp/Empty"
    }

    static func closeAllConnections(x: EmptyRequest) -> EmptyResponse? {
        ROS_INFO("close_all_connections service called, closing connections")
        ConnectionManager.instance.clear(reason: .transportDisconnect)
        return EmptyResponse()
    }

    struct RosgraphMsgs {
        struct Clock: Message {
            init() {
            }

            static let md5sum = "a9c97c1d230cfc112e270351a944ee47"

            static let datatype = "rosgraph_msgs/Clock"

            static let definition = """
                # roslib/Clock is used for publishing simulated time in ROS. \n
                # This message simply communicates the current time.\n
                # For more information, see http://www.ros.org/wiki/Clock\n
                time clock\n
                """

            static let hasHeader = false

        }
    }

    static func clockCallback(msg: RosgraphMsgs.Clock) {
//    Time::setNow(msg->clock);
    }

    static func shutdown() {

        if isShuttingDown.compareAndExchange(expected: false, desired: true) {
            if isStarted {
                TopicManager.instance.shutdown()
                ServiceManager.instance.shutdown()
                ConnectionManager.instance.shutdown()
                XMLRPCManager.instance.shutdown()
            }

            isStarted = false
            isRunning = false
            promise.succeed(result: Void())
        }
    }

}
