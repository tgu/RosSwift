import RosSwift
import Foundation
import RosTime
import StdMsgs
import msgs



// Initiate ros before using any other part of RosSwift

let ros = Ros(argv: &CommandLine.arguments, name: "talker")

// createNode returns nil if the namespace passed is not a valid graph resource name

guard let node = ros.createNode(ns: "", remappings: [:]) else {
    exit(1)
}

// create a service from the Service type

let srv_add_two_ints = custom_msgs.AddTwoInts.advertise(service: "/add_two_ints", node: node) {
    .init(sum: $0.a + $0.b)
}

do {
    // call service from a second node

    let secondNode = ros.createNode()
    let addRequest = custom_msgs.AddTwoInts.Request(a: 34, b: 22)
    let optionalSum = addRequest.call(name: "/add_two_ints", node: secondNode)

    if let sum = optionalSum {
        print("\(addRequest.a) + \(addRequest.b) = \(sum.sum)")
    } else {
        print("AddTwoInts failed")
    }

    // the second node will go out of scope
}

let srv1 = node.advertise(service: "service_adv", srvFunc: caseFlip)

let srv2 = TestStringString.advertise(service: "/echo", node: node) {
    let response = $0.data.uppercased()
    return .init(response)
}

guard let natter_pub = node.advertise(topic: "/natter", message: geometry_msgs.Point.self ) else {
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

guard let chatter_pub = node.advertise(ops: options) else {
    exit(1)
}


// optional publisher

let twist_pub = node.advertise(topic: "/twait", message: geometry_msgs.TwistStamped.self )

let request = TestStringString.Request("request from self")

if let respons : TestStringString.Response = try? Service.call(node: node, serviceName: "echo", req: request).wait() {
    print(respons)
} else {
    print("call returned nil")
}

var parameter: Int = 0

if ros.param.getCached("int", &parameter) {
    print("parameter int = \(parameter)")
}
ros.param.set(key: "~parm", value: ["T":34.3,"I":45.0,"D":0.34])

var rate = Rate(frequency: 1.0)

var j : Int32 = 0
while ros.ok {
    j += 1
    let time = Time.now
    let sm = geometry_msgs.Point(x: 1.0+Float64(j), y: Float64(j), z: sin(Float64(j)))
    natter_pub.publish(message: sm)
    chatter_pub.publish(message: std_msgs.string("Hello \(j)"))

    let header = std_msgs.Header(seq: UInt32(j), stamp: time, frame_id: "test")
    let lin = geometry_msgs.Vector3(x: 1, y: Float64(j), z: sin(Float64(j)))
    let ang = geometry_msgs.Vector3(x: 2.5, y: cos(Float64(j)), z: 4.6 )
    let tw = geometry_msgs.Twist(linear: lin, angular: ang)
    let twist = geometry_msgs.TwistStamped(header: header, twist: tw)

    twist_pub?.publish(message: twist)

    rate.sleep()
}

func caseFlip(req: TestStringString.Request) -> TestStringString.Response? {
    // copy over the request and overwrite the letters with their case-flip
    print("case flip")
    return TestStringString.Response("case flip")
}
