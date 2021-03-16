import RosSwift
import msgs
import StdMsgs


let ros = Ros(argv: &CommandLine.arguments, name: "listener", options: [.anonymousName])
let n = ros.createNode()


class Config {
    @RosParameter(name: "~value", ros: ros)
    var value: Int
}

var config = Config()
config.value = 12


struct B {
    let value : Double
    func chatterCallback(msg: String) {
        print("I [the struct \(config.value)] saw: [\(msg)]")
    }
}

func chatterCallback(msg: String) {
    print("I saw: [\(msg)]")
}

func chatterCallbackEvent(event: MessageEvent<String>) {
    let msg = event.message
    print("I got [\(msg) with header \(event.connectionHeader) at time \(event.receiptTime)")
}


let b = B(value: 99.345)

let vab = n.subscribe(topic: "/chatter", queueSize: 1, callback: chatterCallback)
let bab = n.subscribe(topic: "/chatter", queueSize: 1, callback: b.chatterCallback)
let cab = n.subscribe(topic: "/chatter", queueSize: 1, callback: chatterCallbackEvent)

let sub = n.subscribe(topic: "/natter") { (msg: geometry_msgs.Point) in
    print("accel: [\(msg)]")
}

ros.param.set(key: "/talker/parm", value: 3.34)

n.spinThread()
