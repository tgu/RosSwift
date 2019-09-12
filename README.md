# RosSwift
ROS Robotic Operating System - Swift implementation of core client libraries. Based on roscpp.

RosSwift is a Swift implementation of ROS. It provides a Swift library based on Swift-NIO that enables Swift programmers to interface to ROS. It is tested on OSX and Linux (Raspberry Stretch) and should also work on all platforms that are supported by Swift-NIO.

This is the first version and there are some unimplemented features, basic publishing, subscription, services and parameters should work.

## Compability

Minimum SDK10.12 on OSX

Swift 5 on Raspberry, https://swift-arm.com/2019/01/07/official-swift-arm-community-releases/

## Messages

The package include ``msgbuilder`` that uses calls to ``rosmsg`` to find all defined messages and generate Swift code.

## Example

The following code snippet shows how to model a publisher

```swift
import Foundation
import RosSwift
import RosTime
import StdMsgs


let ros = Ros(argv: &CommandLine.arguments, name: "talker")
let n = ros.createNode()
guard let chatter_pub = n.advertise(topic: "/chatter", message: String.self) else {
    exit(1)
}

var rate = Rate(frequency: 10.0)

var j : Int32 = 0
while ros.ok {
    chatter_pub.publish(message: "Hello \(j)")
    j += 1
    rate.sleep()
}
