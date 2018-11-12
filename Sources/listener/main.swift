import Foundation
import RosSwift
import geometry_msgs
import StdMsgs

func natterCallback(msg: geometry_msgs.Point) {
    print("I heard: [\(msg)]")
}

func chatterCallback(msg: String) {
    print("I saw: [\(msg)]")
}

let future = Ros.initialize(argv: &CommandLine.arguments, name: "listener")


let n = Ros.NodeHandle()

let sub = n.subscribe(topic: "/natter", callback: natterCallback)
let vab = n.subscribe(topic: "/chatter", callback: chatterCallback)

do {
    try future.wait()
} catch {
    print(error.localizedDescription)
}



