# RosSwift
ROS Robotic Operating System - Swift implementation of core client libraries. Based on roscpp.

RosSwift is a Swift implementation of ROS. It provides a Swift library based on Swift-NIO that enables Swift programmers to interface to ROS. It is tested on OSX and Linux (Raspberry Stretch) and should also work on all platforms that are supported by Swift-NIO.

This is the first version and there are some unimplemented features, basic publishing, subscription, services and parameters should work.

## Compability

Minimum SDK10.12 on OSX

Swift 4.2 on Raspberry, https://swift-arm.com/2019/01/07/official-swift-arm-community-releases/

Swift5 and Swift-nio2 support is found in branch Swift5

## Messages

The package include ``msgbuilder`` that uses calls to ``rosmsg`` to find all defined messages and generate Swift code.

## Example

The following code snippet shows how to model a publisher

```swift
import Foundation
import RosSwift
import RosTime
import StdMsgs


let future = Ros.initialize(argv: &CommandLine.arguments, name: "talker")
let n = Ros.NodeHandle()
guard let chatter_pub = n.advertise(topic: "/chatter", message: String.self) else {
    exit(1)
}

var rate = RosTime.Rate(frequency: 10.0)

var j : Int32 = 0
while Ros.ok {
    chatter_pub.publish(message: "Hello \(j)")
    j += 1
    rate.sleep()
}

do {
    try future.wait()
} catch {
    print(error.localizedDescription)
}
```
