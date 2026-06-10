# RosSwift API Reference

This document covers the public API of the three user-facing components:

| Component | Kind | Library you program against |
|---|---|---|
| **RosSwift** | client library | `import RosSwift` |
| **roscore** | master executable | `import rosmaster` (to embed the master) |
| **msgbuilder** | message-generator CLI | `import msgbuilderLib` (to embed generation) |

RosSwift uses Swift concurrency: node creation, publishing, subscribing,
parameters and service calls are `async`. Importing `RosSwift` re-exports
`RosTime`, `StdMsgs`, `msgs`, and `RosNetwork`.

---

# 1. RosSwift (client library)

## 1.1 Node — `Ros`

The entry point. Construct it, then create `NodeHandle`s from it.

```swift
public final class Ros: Sendable {
    public init(name: String, namespace: String = "", remappings: StringStringMap = [:],
                unhandledArgs: [String] = [], options: InitOption = []) throws
    public convenience init(argv: [String], name: String, options: InitOption = []) throws

    public let name: String                 // fully-resolved node name
    public let param: Param                 // parameter-server interface
    public let isRunning: ManagedAtomic<Bool>
    public var ok: Bool                      // true until shutdown

    public func createNode() async -> NodeHandle
    public func createNode(ns: String, remappings: StringStringMap = [:]) async -> NodeHandle?
    public func createNode(parent: NodeHandle, ns: String = "") async -> NodeHandle

    public func getTopics() async throws -> [TopicInfo]
    public func getNodes() async throws -> [String]
    public func getGlobalCallbackQueue() -> AsyncCallbackQueue

    public func spin() async                // process callbacks until shutdown
    public func spinOnce() async            // process ready callbacks, then return
    public func waitForShutdown() async
    public func shutdown()                  // sync teardown
    public func shutdownAsync() async       // awaits in-flight master unregister calls
}
```

| Type | Description |
|---|---|
| `Ros.InitOptions` | `.noSigintHandler`, `.anonymousName`, `.noRosout` |
| `Ros.InitOption` | `Set<InitOptions>` |
| `RosID` | stable node identity (`Hashable, Sendable`) |
| `StringStringMap` | `[String: String]` |
| `TopicInfo` | `{ name, dataType }` returned by `getTopics()` |

`Ros` conforms to `ServiceLifecycle.Service`, so it can run inside a
`ServiceGroup` (`func run() async throws`); otherwise the first `NodeHandle`
starts it lazily.

## 1.2 `NodeHandle`

Created from `Ros`; the object you advertise/subscribe through. Reference-
counted — the node shuts down when the last handle is released.

```swift
public final class NodeHandle: Sendable {
    public var isOK: Bool
    public var namespace: String
    public let unresolvedNamespace: String
    public var remappings: StringStringMap
    public var rosMasterPath: String        // host:port of the master
    public func getCallbackQueue() -> AsyncCallbackQueue
    public func spinThread() async          // delegates to Ros.spin()
}
```

## 1.3 Publishing

```swift
extension NodeHandle {
    public func advertise<M: Message>(
        topic: String, queueSize: UInt = 0,
        connectCall: SubscriberStatusCallback? = nil,
        disconnectCall: SubscriberStatusCallback? = nil,
        latch: Bool = false, message: M.Type,
        tracked_object: TrackableObject? = nil) async -> SpecializedPublisher<M>?

    public func advertise<M: Message>(ops: AdvertiseOptions<M>) async -> SpecializedPublisher<M>?
}

public protocol Publisher: Sendable {
    var topic: String { get }
    var numSubscribers: Int { get }
    func publish(message: Message) async
}
```

`advertise` returns a `SpecializedPublisher<M>` (a `Publisher`). Retain it to
stay advertised. `AdvertiseOptions<M>` is the options-struct form.

```swift
let pub = await node.advertise(topic: "/chatter", message: std_msgs.string.self)
await pub?.publish(message: std_msgs.string("hello"))
```

## 1.4 Subscribing

```swift
extension NodeHandle {
    // callback form
    public func subscribe<M: Message>(
        topic: String, queueSize: UInt32 = 1,
        callback: @escaping @Sendable (M) -> Void,
        tracked_object: TrackableObject? = nil) async -> Subscriber?

    // AsyncStream form
    public func subscribe<M: Message>(topic: String, queueSize: UInt32 = 1,
                                      msg: M.Type) -> AsyncStream<M>
}

public final class Subscriber {
    public let topic: String
    public var publisherCount: Int
    public func unsubscribe()
}
```

Retain the returned `Subscriber` to keep the callback alive; the
`AsyncStream` form lives as long as the stream is consumed.

```swift
let sub = await node.subscribe(topic: "/chatter") { (msg: std_msgs.string) in print(msg) }
for await msg in node.subscribe(topic: "/chatter", msg: std_msgs.string.self) { print(msg) }
```

## 1.5 Services

**Advertise** (server side):

```swift
extension ServiceProt {
    public static func advertise(service: String, node: NodeHandle,
        handler: @escaping @Sendable (Request) -> Response) async -> ServiceServer?
}
extension NodeHandle {
    public func advertise<MReq: ServiceRequestMessage, MRes: ServiceResponseMessage>(
        service: String, srvFunc: @escaping @Sendable (MReq) -> MRes?,
        tracked_object: TrackableObject? = nil) async -> ServiceServer?
}
```

**Call** (client side):

```swift
public enum Service {
    public static func call<MReq: ServiceMessage, MRes: ServiceMessage>(
        node: NodeHandle, serviceName: String, req: MReq) async throws -> MRes
    public static func waitForService(ros: Ros, serviceName: String,
        timeout: RosDuration = RosDuration()) async -> Bool
    public static func exists(ros: Ros, serviceName: String, printFailureReason: Bool) async -> Bool
}

extension ServiceRequestMessage {
    public func call(name: String, node: NodeHandle) async -> ServiceType.Response?
}
```

**Persistent client**:

```swift
extension NodeHandle {
    public func serviceClient(service: String, md5sum: String,
        persistent: Bool = false, headerValues: StringStringMap? = nil) async -> ServiceClient
}

public final class ServiceClient {
    public func call<MReq: ServiceMessage, MRes: ServiceMessage>(req: MReq) async throws -> MRes
    public func waitForExistence(ros: Ros,
        timeout: RosDuration = RosDuration(seconds: -1)) async -> Bool
    public func exists() async -> Bool
    public func getService() -> String
}
```

```swift
let resp = try await Service.call(node: node, serviceName: "/add", req: AddTwoInts.Request(a: 1, b: 2))
```

## 1.6 Parameters — `Ros.param` (`Param` actor)

```swift
public actor Param: Sendable {
    public func get<T>(_ key: String, _ value: inout T) async -> Bool
    public func getCached<T>(_ key: String, default: T,
        completion: SubscribedParameterHandler? = nil) async -> T
    public func set<T>(key: String, value: T) async
    public func has(key: String) async -> Bool
    public func del(key: String) async -> Bool
    public func search(key: String, result: inout String) async -> Bool
    public func getParameterNames() async throws -> [String]
    public func param<T>(name: String, defaultValue: T) async -> T
}
```

`NodeHandle` also exposes namespace-resolving conveniences: `get(parameter:value:)`,
`set(parameter:value:)`, `has(parameter:)`, `search(parameter:result:)`,
`param(name:value:defaultValue:)`.

```swift
let count: Int = await ros.param.getCached("count", default: 10)
await ros.param.set(key: "rate", value: 5.0)
```

`@RosParameter` is a property-wrapper form (`public struct RosParameter<Value>`).

## 1.7 Time, durations & rates (`RosTime`, re-exported)

| Type | Notes |
|---|---|
| `Time`, `WallTime` | timestamps (ROS time / wall clock) |
| `RosDuration`, `WallDuration` | durations; `.sleep() async` |
| `Rate` | loop rate |

```swift
public struct Rate {
    public init(frequency: Double)
    public init(duration: RosDuration)
    public func sleep() async -> Bool
    public func reset()
    public func cycleTime() -> RosDuration
}
```

```swift
let rate = Rate(frequency: 10)
while ros.ok { /* work */ await rate.sleep() }
```

## 1.8 Timers (on `NodeHandle`)

```swift
public func createTimer(period: RosDuration, oneshot: Bool = false,
    autostart: Bool = true, trackedObject: TrackableObject? = nil,
    callback: @escaping TimerCallback) async -> Timer
public func createTimer(rate: Rate, ...) async -> Timer
public func createWallTimer(period: WallDuration, ...,
    callback: @escaping @Sendable (WallTimerEvent) -> Void) async -> WallTimer
```

## 1.9 Messages (`StdMsgs`, re-exported)

Generated message structs conform to these protocols:

| Protocol | Role |
|---|---|
| `Message` | `BinaryCodable, Sendable` base for all messages |
| `MessageWithHeader` | message carrying a `std_msgs/Header` |
| `ServiceProt` | a service (`Request`/`Response` pair) |
| `ServiceMessage` / `ServiceRequestMessage` / `ServiceResponseMessage` | service payloads |

Message packages (e.g. `std_msgs`, `geometry_msgs`, `sensor_msgs`) come from
the re-exported `msgs` module.

## 1.10 Logging

RosSwift logs through [swift-log](https://github.com/apple/swift-log).

```swift
public enum RosLogging {
    public static func bootstrap(subsystem: String = "org.ros.rosswift")
}

#if canImport(os)   // Apple platforms
public struct OSLogHandler: LogHandler {
    public init(label: String, subsystem: String)
}
#endif
```

`RosLogging.bootstrap()` installs Apple unified logging (`os.Logger`) on Apple
platforms and `StreamLogHandler` (stdout) on Linux. Free logging functions:
`ROS_INFO`, `ROS_WARNING`, `ROS_ERROR`, and their `*_STREAM` variants. Levels
are adjustable at runtime via the node's `~set_logger_level` service.

---

# 2. roscore / the master (`rosmaster`)

`roscore` is an executable that runs the ROS master. To embed the master, use
the `rosmaster` library.

```swift
public let defaultMasterPort = 11311

public final class Master: NSObject, NetServiceDelegate, @unchecked Sendable {
    public init(host: String, port: Int = defaultMasterPort, advertise: Bool = true)

    public let host: String
    public let requestedPort: Int
    public var port: Int        // actually-bound port (when requestedPort == 0)
    public var address: String  // "http://host:port"

    public func run() async throws   // ServiceLifecycle.Service conformance
}
```

`Master` is a `ServiceLifecycle.Service`; run it inside a `ServiceGroup`:

```swift
import rosmaster
import ServiceLifecycle

let master = Master(host: network.gHost, port: defaultMasterPort)
let group = ServiceGroup(services: [master],
                         gracefulShutdownSignals: [.sigterm, .sigint], logger: logger)
try await group.run()
```

The `rosmaster` module also exposes some general-purpose utilities used by the
master implementation: `Multimap<Key,Value>`, `RegistrationManager`,
`XMLRPCServer`, and terminal-styling helpers (`TerminalStyle`, `TerminalColor`).

`RosNetwork` provides `appVersion` and host/URI helpers (`RosNetwork`,
`determineHost()`, `splitURI(uri:)`).

---

# 3. msgbuilder (message generator)

`msgbuilder` converts ROS `.msg` / `.srv` definitions into Swift structs.

## 3.1 CLI

```
msgbuilder [-o | -b | -t] source destination
msgbuilder [-o | -b | -t] -r destination
```

| Option | Meaning |
|---|---|
| `-o` | overwrite existing files |
| `-r` | convert all messages found by `rosmsg list` |
| `-b` | regenerate builtins |
| `-t` | do not embed messages in an enum namespace |

`source` is a directory of message definitions; `destination` is where the
generated Swift files are written.

## 3.2 `msgbuilderLib`

To drive generation programmatically:

```swift
public final class MsgContext {
    public init(useBuiltin: Bool, embed: Bool = true)
    public func load_dir(path: URL, package_name: String)
    public func addMsg(with content: String, full_name: String,
                       messageType: MessageType, generate: Bool) -> MsgSpec?
    public func genAllMessages(to destination: URL)
}

public enum MessageType: String { /* .message, .service, ... */ }
public struct MsgSpec: BaseMsg { /* parsed message specification */ }
```

Supporting helpers: `Shell` (`rosmsg(_:)` wrapper around the ROS CLI), and the
MD5 utilities `md5_(_:) -> Digest` / `encodeMD5(digest:) -> String` used to
compute message checksums.

---

## See also

- `README.md` — installation, quick-start examples, and the `ServiceGroup`
  lifecycle pattern.
- Source doc comments — most public declarations carry `///` documentation
  usable by Xcode Quick Help and DocC.
