import RosSwift
import Foundation
import ServiceLifecycle
import Logging

// Route swift-log to Apple unified logging on Apple platforms, stdout on Linux.
RosLogging.bootstrap()

let logger = Logger(label: "talker")

// .noSigintHandler so ServiceGroup owns SIGINT/SIGTERM.
var args = CommandLine.arguments
let ros = try Ros(argv: args, name: "talker", options: [.noSigintHandler])

// createNode returns nil if the namespace passed is not a valid graph resource name

guard let node = await ros.createNode(ns: "", remappings: [:]) else {
    exit(1)
}

// create a service from the Service type

let srv_add_two_ints = await custom_msgs.AddTwoInts.advertise(service: "/add_two_ints", node: node) {
    .init(sum: $0.a + $0.b)
}

do {
    // call service from a second node

    let secondNode = await ros.createNode()
    let addRequest = custom_msgs.AddTwoInts.Request(a: 34, b: 22)
    let optionalSum = await addRequest.call(name: "/add_two_ints", node: secondNode)

    if let sum = optionalSum {
        print("\(addRequest.a) + \(addRequest.b) = \(sum.sum)")
    } else {
        print("AddTwoInts failed")
    }

    // the second node will go out of scope
}

let srv1 = await node.advertise(service: "service_adv", srvFunc: caseFlip)

let srv2 = await TestStringString.advertise(service: "/echo", node: node) {
    let response = $0.data.uppercased()
    return .init(response)
}

guard let natter_pub = await node.advertise(topic: "/natter", message: geometry_msgs.Point.self ) else {
    exit(1)
}

func subscriberCallback(_ link: SingleSubscriberPublisher) -> Void {
    print("Subscriber [\(link.callerId)] has joined")
    link.publish(Int32(999))
}

let options = AdvertiseOptions(topic: "/chatter",
                               std_msgs.string.self,
                               connectCall: subscriberCallback,
                               disconnectCall: { print("Subscriber [\($0.callerId)] has left")} )

guard let chatter_pub = await node.advertise(ops: options) else {
    exit(1)
}


// optional publisher

let twist_pub = await node.advertise(topic: "/twait", message: geometry_msgs.TwistStamped.self )

let request = TestStringString.Request("request from self")

if let respons : TestStringString.Response = try? await Service.call(node: node, serviceName: "echo", req: request) {
    print(respons)
} else {
    print("call returned nil")
}

var parameter: Int = await ros.param.getCached("int", default: 12)

await ros.param.set(key: "~parm", value: ["T":34.3,"I":45.0,"D":0.34])

// ServiceGroup owns the signal traps. The publish loop and spin run as sibling
// tasks; when shutdown is signaled, Ros.run() returns, `ros.ok` flips false,
// and the loops exit naturally.
let group = ServiceGroup(
    services: [ros],
    gracefulShutdownSignals: [.sigterm, .sigint],
    logger: logger
)
try await withThrowingTaskGroup(of: Void.self) { tg in
    tg.addTask { try await group.run() }
    tg.addTask { await ros.spin() }
    tg.addTask {
        let rate = Rate(frequency: 1.0)
        var j: Int32 = 0
        while ros.ok {
            j += 1
            let time = Time.now
            let sm = geometry_msgs.Point(x: 1.0+Float64(j), y: Float64(j), z: sin(Float64(j)))
            await natter_pub.publish(message: sm)
            await chatter_pub.publish(message: std_msgs.string("Hello \(j)"))

            let header = std_msgs.Header(seq: UInt32(j), stamp: time, frame_id: "test")
            let lin = geometry_msgs.Vector3(x: 1, y: Float64(j), z: sin(Float64(j)))
            let ang = geometry_msgs.Vector3(x: 2.5, y: cos(Float64(j)), z: 4.6 )
            let tw = geometry_msgs.Twist(linear: lin, angular: ang)
            let twist = geometry_msgs.TwistStamped(header: header, twist: tw)

            await twist_pub?.publish(message: twist)

            await rate.sleep()
        }
    }
    _ = try await tg.next()
    tg.cancelAll()
}

func caseFlip(req: TestStringString.Request) -> TestStringString.Response? {
    // copy over the request and overwrite the letters with their case-flip
    print("case flip")
    return TestStringString.Response("case flip")
}
