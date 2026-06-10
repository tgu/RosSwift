import RosSwift
import ServiceLifecycle
import Logging

// Route swift-log to Apple unified logging on Apple platforms, stdout on Linux.
RosLogging.bootstrap()

let logger = Logger(label: "listener")

// .noSigintHandler so ServiceGroup owns SIGINT/SIGTERM.
let ros = try Ros(argv: CommandLine.arguments, name: "listener", options: [.anonymousName, .noSigintHandler])
let n = await ros.createNode()

func chatterCallback(msg: String) {
    print("I saw: [\(msg)]")
}

func chatterCallbackEvent(event: MessageEvent<String>) {
    let msg = event.message
    print("I got [\(msg) with header \(event.connectionHeader) at time \(event.receiptTime)")
}

let vab = await n.subscribe(topic: "/chatter", queueSize: 0, callback: chatterCallback)
let cab = await n.subscribe(topic: "/chatter", queueSize: 0, callback: chatterCallbackEvent)

let sub = await n.subscribe(topic: "/natter") { (msg: geometry_msgs.Point) in
    print("accel: [\(msg)]")
}

let stream = n.subscribe(topic: "/chatter", queueSize: 10, msg: String.self)

Task(name: "snagger") {
    for await snag in stream {
        print("snag = \(snag)")
    }
}

await ros.param.set(key: "/talker/parm", value: 3.34)

// ServiceGroup owns the signal traps and orchestrates lifecycle. We run the
// spin loop as a sibling task so callbacks fire while the node is alive; when
// shutdown is signaled, Ros.run() returns and we cancel the spin task.
let group = ServiceGroup(
    services: [ros],
    gracefulShutdownSignals: [.sigterm, .sigint],
    logger: logger
)
try await withThrowingTaskGroup(of: Void.self) { tg in
    tg.addTask { try await group.run() }
    tg.addTask { await ros.spin() }
    _ = try await tg.next()
    tg.cancelAll()
}
