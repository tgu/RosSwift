import RosSwift
import Foundation
import RosTime
import StdMsgs
import geometry_msgs


struct AddTwoIntsRequest: ServiceMessage {
    init() {
        a = 0
        b = 0
    }

    static var srvMd5sum: String = AddTwoInts.md5sum
    static var srvDatatype: String = AddTwoInts.datatype
    static var md5sum: String = "36d09b846be0b371c5f190354dd3153e"
    static var datatype: String = "beginner_tutorials/AddTwoIntsRequest"
    static var hasHeader: Bool = false
    static var definition: String = "int64 a\nint64 b"

    var a : Int64
    var b : Int64

    init(a: Int64, b: Int64) {
        self.a = a
        self.b = b
    }
}

struct AddTwoIntsResponse: ServiceMessage {
    init() {
        sum = 0
    }

    static var srvMd5sum: String = AddTwoInts.md5sum
    static var srvDatatype: String = AddTwoInts.datatype
    static var md5sum: String = "b88405221c77b1878a3cbbfff53428d7"
    static var datatype: String = "beginner_tutorials/AddTwoIntsResponse"
    static var hasHeader: Bool = false
    static var definition: String = "int64 sum"

    var sum : Int64
}

struct AddTwoInts {
    typealias Request = AddTwoIntsRequest

    typealias Response = AddTwoIntsResponse

    public static var datatype: String = "beginner_tutorials/AddTwoInts"
    public static var md5sum = "6a2e34150c00229791cc89ff309fff21"
}

func caseFlip(req: TestStringString.Request) -> TestStringString.Response? {
    // copy over the request and overwrite the letters with their case-flip
    print("case flip")
    return TestStringString.Response("case flip")
}

func echo(req: TestStringString.Request) -> TestStringString.Response? {
    // copy over the request and overwrite the letters with their case-flip

    let response = req.data.uppercased()

    return TestStringString.Response(response)
}

let ros = Ros(argv: &CommandLine.arguments, name: "talker")


var keys = [String]()

guard let n = ros.createNode(ns: "") else {
    exit(1)
}

let req = AddTwoIntsRequest(a: 34, b: 22)
if let res : AddTwoIntsResponse = try? Service.call(node: n, serviceName: "/add_two_ints", req: req).wait() {
    print("\(req.a) + \(req.b) = \(res.sum)")
} else {
    print("AddTwoIntsResponse failed")
}


let srv1 = n.advertise(service: "service_adv", srvFunc: caseFlip)
let srv2 = n.advertise(service: "echo") { (req: TestStringString.Request) -> TestStringString.Response? in
    let response = req.data.uppercased()
    return .init(response)
}


guard let natter_pub = n.advertise(topic: "/natter", message: geometry_msgs.Point.self ) else {
    exit(1)
}

func subscriberCallback(_ link: SingleSubscriberPublisher) -> Void {
    print("Subscriber [\(link.callerId)] has joined")
    link.publish(Int32(999))
}

func subscriberLeavingCallback(_ link: SingleSubscriberPublisher) -> Void {
    print("Subscriber [\(link.callerId)] has left")
}


var options = AdvertiseOptions(topic: "/chatter",std_msgs.string.self)
options.connectCallBack = subscriberCallback
options.disconnectCallBack = subscriberLeavingCallback

guard let chatter_pub = n.advertise(ops: options) else {
    exit(1)
}
guard let twist_pub = n.advertise(topic: "/twait", message: geometry_msgs.TwistStamped.self ) else {
    exit(1)
}


let request = TestStringString.Request("request from self")

if let respons : TestStringString.Response = try? Service.call(node: n, serviceName: "echo", req: request).wait() {
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

    twist_pub.publish(message: twist)

    rate.sleep()
}

//do {
//    try future.wait()
//} catch {
//    print(error.localizedDescription)
//}
