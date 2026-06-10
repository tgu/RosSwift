# RosSwift

[![](https://img.shields.io/endpoint?url=https%3A%2F%2Fswiftpackageindex.com%2Fapi%2Fpackages%2Ftgu%2FRosSwift%2Fbadge%3Ftype%3Dswift-versions)](https://swiftpackageindex.com/tgu/RosSwift)

[![](https://img.shields.io/endpoint?url=https%3A%2F%2Fswiftpackageindex.com%2Fapi%2Fpackages%2Ftgu%2FRosSwift%2Fbadge%3Ftype%3Dplatforms)](https://swiftpackageindex.com/tgu/RosSwift)

ROS Robotic Operating System - Swift implementation of core client libraries. Based on roscpp.

RosSwift is a Swift implementation of ROS. It provides a Swift library based on Swift-NIO that enables Swift programmers to interface to ROS. It is tested on macOS and Linux and should also work on all platforms supported by Swift-NIO. There is also an iOS example app included.

There are some unimplemented features, basic publishing, subscription, services and parameters should work.

## Compatibility

- Swift 6.2+
- macOS 15.4+, iOS 18.4+, tvOS 18.4+
- Linux (Ubuntu)

It can be necessary to allow arbitrary loads in the Info.plist file for iOS and macOS:

```xml
<key>NSAppTransportSecurity</key>
<dict>
        <key>NSAllowsArbitraryLoads</key>
        <true/>
</dict>
```

## Installation

Add RosSwift to your `Package.swift`:

```swift
dependencies: [
    .package(url: "https://github.com/tgu/RosSwift.git", from: "2.0.0")
],
targets: [
    .executableTarget(name: "MyNode", dependencies: [
        .product(name: "RosSwift", package: "RosSwift")
    ])
]
```

## Concurrency

RosSwift uses Swift concurrency (`async`/`await`) throughout. Node creation, publishing, subscribing, parameters, and service calls are all `async` operations, and the core types are `Sendable`. Shared state is protected with `Synchronization.Mutex` and atomics; the master handler is an `actor`.

Service calls return their response directly:

```swift
let response: AddTwoInts.Response = try await Service.call(
    node: node, serviceName: "/add_two_ints", req: request)
```

### Structured lifecycle

`Ros` conforms to [`ServiceLifecycle.Service`](https://github.com/swift-server/swift-service-lifecycle), so a node can be run inside a `ServiceGroup` that owns signal handling and orchestrates graceful startup/shutdown:

```swift
import ServiceLifecycle

let ros = try Ros(argv: CommandLine.arguments, name: "talker", options: [.noSigintHandler])
let node = await ros.createNode()
// ... advertise / subscribe ...

let group = ServiceGroup(
    services: [ros],
    gracefulShutdownSignals: [.sigterm, .sigint],
    logger: logger)

try await withThrowingTaskGroup(of: Void.self) { tg in
    tg.addTask { try await group.run() }   // runs until SIGINT/SIGTERM
    tg.addTask { await ros.spin() }        // process callbacks
    _ = try await tg.next()
    tg.cancelAll()
}
```

On shutdown the node awaits its in-flight master `unregister*` calls (`shutdownAsync()`) so the master sees a clean teardown. Without a `ServiceGroup`, the first `NodeHandle` you create starts the node lazily, and releasing the last one (or calling `ros.shutdown()`) tears it down.

## Messages

The package includes `msgbuilder` to generate code for custom messages.

## Master

The package includes `roscore`, a Swift implementation of the ROS master. The master advertises its presence with Bonjour on Apple devices and with Avahi on Linux devices. A client will search for the master automatically unless the URL is specified with remapping or with the environment variable `ROS_MASTER_URI`:

```swift
let ros = try Ros(name: "phone", remappings: ["__master": "http://10.0.1.23:11311"])
```

For service discovery in an iOS client app, add `NSBonjourServices` to your Info.plist:

```xml
<key>NSBonjourServices</key>
<array>
  <string>_ros._tcp</string>
</array>
```

## Examples

### Publisher

```swift
import RosSwift
import Foundation

let ros = try Ros(argv: CommandLine.arguments, name: "talker")
guard let node = await ros.createNode(ns: "", remappings: [:]) else {
    exit(1)
}

guard let chatter_pub = await node.advertise(topic: "/chatter", message: std_msgs.string.self) else {
    exit(1)
}

var rate = Rate(frequency: 1.0)

var j: Int32 = 0
while ros.ok {
    j += 1
    await chatter_pub.publish(message: std_msgs.string("Hello \(j)"))
    await rate.sleep()
}
```

### Listener

```swift
import RosSwift

let ros = try Ros(argv: CommandLine.arguments, name: "listener", options: [.anonymousName])
let n = await ros.createNode()

let vab = await n.subscribe(topic: "/chatter", queueSize: 0) { (msg: String) in
    print("I saw: [\(msg)]")
}

// Subscribe using AsyncStream
let stream = n.subscribe(topic: "/chatter", queueSize: 10, msg: String.self)
Task {
    for await msg in stream {
        print("stream: \(msg)")
    }
}

await n.spinThread()
```

### Services

```swift
// Advertise a service
let srv = await custom_msgs.AddTwoInts.advertise(service: "/add_two_ints", node: node) {
    .init(sum: $0.a + $0.b)
}

// Call a service
let request = custom_msgs.AddTwoInts.Request(a: 34, b: 22)
if let response = await request.call(name: "/add_two_ints", node: node) {
    print("\(request.a) + \(request.b) = \(response.sum)")
}
```

### iOS App

RosSwift works with SwiftUI. See the included example app in `Examples/iOSApp/` for a full implementation. The basic pattern uses an `@Observable` wrapper:

```swift
import SwiftUI
import RosSwift

@Observable class NodeWrapper {
    var node: NodeHandle?

    var isRunning: Bool {
        node?.isOK ?? false
    }

    var task: Task<Void, Never>?

    func connect(_ master: String) async throws {
        let ros = try Ros(name: "iPhone", remappings: ["__master": master])
        node = await ros.createNode()

        task = Task.detached(priority: .background) { [weak self] in
            await self?.node?.spinThread()
        }
    }

    func disconnect() {
        if isRunning {
            task?.cancel()
            task = nil
            node = nil
        }
    }
}
```
