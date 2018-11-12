import Foundation
import StdMsgs
import RosTime
import HeliumLogger
import LoggerAPI
import NIO
import NIOConcurrencyHelpers

public typealias M_string = [String : String]

struct TransportTCP {
    static var s_use_keepalive = false
    static var s_use_ipv6 = false
}


func basicSigintHandler(signal: Int32) {
    ROS_INFO("SIGINT")
    Ros.requestShutdown()
}

func atexitCallback() {
    if Ros.ok && !Ros.isShuttingDown {
        ROS_DEBUG("shutting down due to exit() or end of main() without cleanup of all NodeHandles")
        Ros.g_started = false
        Ros.shutdown()
    }
}

func amIBeingDebugged() -> Bool {
    #if os(OSX)
    var info = kinfo_proc()
    var mib : [Int32] = [CTL_KERN, KERN_PROC, KERN_PROC_PID, getpid()]
    var size = MemoryLayout<kinfo_proc>.stride
    let junk = sysctl(&mib, UInt32(mib.count), &info, &size, nil, 0)
    assert(junk == 0, "sysctl failed")
    return (info.kp_proc.p_flag & P_TRACED) != 0
    #else
    return false
    #endif
}

public struct Ros {
        public enum init_options {
            case NoSigintHandler
            case AnonymousName
            case NoRosout
        }

    public typealias InitOption = Set<init_options>


    static var g_rosout_appender : ROSOutAppender?
    static var fileLog: FileLog? = nil
    static var g_init_options = InitOption()
    static var g_shutdown_requested = false
    static var g_shutting_down = Atomic<Bool>(value: false)
    static var g_ok = false
    static var g_started = false
    static var g_initialized = false
    static var g_atexit_registered = false
    static let logg = HeliumLogger(.debug)

    static var isInitialized:  Bool {
        return g_initialized
    }

    static var isShuttingDown: Bool {
        return g_shutting_down.load()
    }

    static func requestShutdown() {
        g_shutdown_requested = true
        shutdown()
    }


    static func shutdownCallback(params: XmlRpcValue) -> XmlRpcValue  {
        var num_params = 0
        switch params.getType() {
        case  .array(let a):
            num_params = a.count
        default:
            break
        }
        if num_params > 1 {
            let reason = params[1]
            ROS_INFO("Shutdown request received.")
            ROS_INFO("Reason given for shutdown: \(reason)")
            // we have to avoid calling wait inside an EventLoop
            DispatchQueue(label: "shutdown").async {
                requestShutdown()
            }
        }

        return xmlrpc.responseInt(code: 1, msg: "", response: 0)
    }



    static var isStarted: Bool {
        return g_started
    }

    public static var ok : Bool {
        return g_ok
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

    public static func initialize(argv: inout [String], name: String, options: InitOption = .init()) -> EventLoopFuture<Void> {

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

        var remappings = M_string()
        var unhandled = [String]()

        for arg in argv {
            if let pos = arg.range(of: ":=") {
                let local_name = String(arg.prefix(upTo: pos.lowerBound))
                let external_name = String(arg.suffix(from: pos.upperBound))
                ROS_DEBUG("remap \(local_name) => \(external_name)")
                remappings[local_name] = external_name
            } else {
                unhandled.append(arg)
            }
        }
        argv = unhandled
        return initialize(remappings: remappings, name: name, options: options)
    }

    /** Alternate ROS initialization function.

       Alternate ROS initialization function

    - Parameter remappings: A map<string, string> where each one constitutes a name remapping, or one of the special remappings like __name, __master, __ns, etc.
    - Parameter name: Name of this node.  The name must be a base name, ie. it cannot contain namespaces.
    - Parameter options: [optional] Options to start the node with (a set of bit flags from \ref ros::init_options)
     */

    static let promise : EventLoopPromise<Void> = thread_group.next().newPromise()


    public static func initialize(remappings: M_string, name: String, options: InitOption) -> EventLoopFuture<Void> {


        if !g_atexit_registered {
            g_atexit_registered = true
            atexit(atexitCallback)
        }


        g_init_options = options
        g_ok = true

        check_ipv6_environment();
        network.initialize(remappings: remappings)
        Master.shared.initialize(remappings: remappings)
        this_node.initialize(name: name, remappings: remappings, options: options)
        fileLog = FileLog(remappings: remappings)
        param.initialize(remappings: remappings)

        g_initialized = true

        return Ros.promise.futureResult
    }

    static func check_ipv6_environment() {
        if let env_ipv6 = getenv("ROS_IPV6") {
            let env = String(utf8String: env_ipv6)
            let use_ipv6 = env == "on"
        }
    }

    func removeROSArgs(argv: [String]) -> [String] {
        return argv.filter { $0.contains(":=") }
    }

    public static func waitForShutdown() {
        while ok {
            _ = RosTime.WallDuration(seconds: 0.05).sleep()
        }
        promise.succeed(result: Void())
    }


    private static func kill() {
        ROS_ERROR("Caught kill, stopping...")
        DispatchQueue.main.async {
            g_shutdown_requested = true
            requestShutdown()
        }
    }


    static func start() {
        ROS_INFO("starting Ros")
        if isStarted {
            return
        }

        g_shutdown_requested = false
        g_started = true
        g_ok = true
        var enable_debug = false

        let _ = param.param(param_name: "/tcp_keepalive", param_val: &TransportTCP.s_use_keepalive, default_val: TransportTCP.s_use_keepalive)

        guard XMLRPCManager.instance.bind(function_name: "shutdown", cb: shutdownCallback) else {
            fatalError("Could not bind function")
        }

        initInternalTimerManager()

        TopicManager.instance.start()
        ServiceManager.instance.start()
        Ros.ConnectionManager.instance.start()
        XMLRPCManager.instance.start()

        if !g_init_options.contains(.NoSigintHandler) {
            signal(SIGINT,basicSigintHandler)
            signal(SIGTERM,basicSigintHandler)
        }

        RosTime.Time.initialize()

        if !g_init_options.contains(.NoRosout) {
            let rosout_appender = ROSOutAppender()
            console.register_appender(appender: rosout_appender)
            g_rosout_appender = rosout_appender
        }

        let _ = ServiceManager.instance.advertiseService(.init(service: "~debug/close_all_connections", callback: closeAllConnections))
        let _ = ServiceManager.instance.advertiseService(.init(service: "~set_logger_level", callback: setLoggerLevel))

        if g_shutting_down.load() { return }

        if enable_debug {
            let _ = ServiceManager.instance.advertiseService(.init(service: "~debug/close_all_connections", callback: closeAllConnections))
        }

        var use_sim_time = false
        let _ = param.param(param_name: "/use_sim_time", param_val: &use_sim_time, default_val: use_sim_time)

        if use_sim_time {
            RosTime.Time.setNow(RosTime.Time())
        }

        if use_sim_time {
            let ops = SubscribeOptions(topic: "/clock", callback: clockCallback)
            if !TopicManager.instance.subscribeWith(options: ops) {
                ROS_ERROR("could not subscribe to /clock")
            }
        }

        if g_shutting_down.load() {
            return
        }

        ROS_INFO("Started node [\(Ros.this_node.getName())], pid [\(getpid())], bound on [\(network.getHost())], xmlrpc port [\(XMLRPCManager.instance.serverPort)], tcpros port [\(Ros.ConnectionManager.instance.getTCPPort())], using [real] time")

    }

    struct Logger: Message {

        static let md5sum = "a6069a2ff40db7bd32143dd66e1f408e"
        static let datatype = "roscpp/Logger"
        static let hasHeader = false
        static let definition = "string name\nstring level\n"

        let name : String
        let level : String

        init() {
            name = ""
            level = ""
        }

    }

    struct GetLoggersRequest: ServiceMessage {
        static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
        static let srv_md5sum = GetLoggers.md5sum
        static let srv_datatype = GetLoggers.datatype
        static let datatype = "roscpp/GetLoggersRequest"
        static let hasHeader = false
        static let definition = "\n"
    }

    struct GetLoggersResponse: ServiceMessage {
        static let md5sum = "32e97e85527d4678a8f9279894bb64b0"
        static let srv_md5sum = GetLoggers.md5sum
        static let srv_datatype = GetLoggers.datatype
        static let datatype = "roscpp/GetLoggersResponse"
        static let hasHeader = false
        static let definition = """
            Logger[] loggers

            ================================================================================\n\
            MSG: roscpp/Logger
            string name
            string level
            """

        let loggers : [Logger]

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
        static let srv_md5sum = SetLoggerLevel.md5sum
        static let srv_datatype = SetLoggerLevel.datatype


        let logger : String
        let level : String

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
        static let srv_md5sum = SetLoggerLevel.md5sum
        static let srv_datatype = SetLoggerLevel.datatype
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
        static let srv_md5sum = Empty.md5sum
        static let srv_datatype = Empty.datatype
        static let datatype = "roscpp/EmptyRequest"
        static let hasHeader = false
        static let definition = "\n"
    }

    struct EmptyResponse: ServiceMessage {
        static let md5sum = "d41d8cd98f00b204e9800998ecf8427e"
        static let srv_md5sum = Empty.md5sum
        static let srv_datatype = Empty.datatype
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
        ConnectionManager.instance.clear(reason: .TransportDisconnect)
        return EmptyResponse()
    }

    struct rosgraph_msgs {
        struct Clock: Message {
//            func deserialize(from: inout StreamBuffer) {
//                fatalError()
//            }

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

//            func serialize() -> SerializedMessage {
//                return SerializedMessage(msg: self, buffer: [UInt8]() )
//            }


        }
    }

    static func clockCallback(msg: rosgraph_msgs.Clock) {
//    Time::setNow(msg->clock);
    }

    static func shutdown()  {

        if g_shutting_down.compareAndExchange(expected: false, desired: true) {
            if (g_started)  {
                TopicManager.instance.shutdown()
                ServiceManager.instance.shutdown()
                ConnectionManager.instance.shutdown()
                XMLRPCManager.instance.shutdown()
            }
            
            g_started = false
            g_ok = false
            promise.succeed(result: Void())
        }
    }



}
