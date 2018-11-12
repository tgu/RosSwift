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

    static var srv_md5sum: String = AddTwoInts.md5sum
    static var srv_datatype: String = AddTwoInts.datatype
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

    static var srv_md5sum: String = AddTwoInts.md5sum
    static var srv_datatype: String = AddTwoInts.datatype
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
    print("echo")
    return TestStringString.Response(req.data)
}

let future = Ros.initialize(argv: &CommandLine.arguments, name: "talker")


var keys = [String]()

let n = Ros.NodeHandle()

let req = AddTwoIntsRequest(a: 34, b: 22)
do {
    if let res : AddTwoIntsResponse = try service.call(service_name: "/add_two_ints", req: req).wait() {
        print("\(req.a) + \(req.b) = \(res.sum)")
    }
} catch {
    print("AddTwoIntsResponse failed: \(error)" )
}


let srv1 = n.advertiseService(service: "service_adv", srv_func: caseFlip)
let srv2 = n.advertiseService(service: "echo", srv_func: echo)


guard let natter_pub = n.advertise(topic: "/natter", message: geometry_msgs.Point.self ) else {
    exit(1)
}

func subscriberCallback(_ link: SingleSubscriberPublisher) -> Void {
    print("Subscriber \(link) has joined")
    link.publish(Int32(999))
}

func subscriberLeavingCallback(_ link: SingleSubscriberPublisher) -> Void {
    print("Subscriber \(link) has left")
}


var options = AdvertiseOptions(topic: "/chatter",std_msgs.string.self)
options.connect_cb = subscriberCallback
options.disconnect_cb = subscriberLeavingCallback
options.latch = true

guard let chatter_pub = n.advertise(ops: options) else {
    exit(1)
}
guard let twist_pub = n.advertise(topic: "/twait", message: geometry_msgs.TwistStamped.self ) else {
    exit(1)
}


let request = TestStringString.Request("request from self")

do {
    if let respons : TestStringString.Response = try service.call(service_name: "echo", req: request).wait() {
        print(respons)
    } else {
        print("call returned nil")
    }
} catch {
    print("no response: \(error)")
}

var j : Int32 = 0
while Ros.ok {
    j += 1
    let time = RosTime.Time.now()
    let sm = geometry_msgs.Point(x: 1.0+Float64(j), y: Float64(j), z: sin(Float64(j)))
    natter_pub.publish(message: sm)
    chatter_pub.publish(message: std_msgs.string("Hello \(j)"))
    usleep(500_000)

    let header = std_msgs.header(seq: UInt32(j), stamp: time, frameID: 2)
    let lin = geometry_msgs.Vector3(x: 1, y: Float64(j), z: sin(Float64(j)))
    let ang = geometry_msgs.Vector3(x: 2.5, y: cos(Float64(j)), z: 4.6 )
    let tw = geometry_msgs.Twist(linear: lin, angular: ang)
    let twist = geometry_msgs.TwistStamped(header: header, twist: tw)

    twist_pub.publish(message: twist)

    usleep(500_000)
}

do {
    try future.wait()
} catch {
    print(error.localizedDescription)
}
