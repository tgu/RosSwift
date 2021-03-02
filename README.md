# RosSwift
ROS Robotic Operating System - Swift implementation of core client libraries. Based on roscpp.

RosSwift is a Swift implementation of ROS. It provides a Swift library based on Swift-NIO that enables Swift programmers to interface to ROS. It is tested on OSX and Linux (Raspberry Stretch, Ubuntu) and should also work on all platforms that are supported by Swift-NIO.

There are some unimplemented features, basic publishing, subscription, services and parameters should work.

## Compability

Minimum SDK10.14 on OSX, also works on iOS14

Swift 5.3 on Raspberry, tested with Ubuntu 20.10

It can be necessary to allow arbitary loads in the Info.plist file for iOS and macOS

```
<key>NSAppTransportSecurity</key>
<dict>
        <key>NSAllowsArbitraryLoads</key>
        <true/>
</dict>
```


## Messages

The package include ``msgbuilder`` to generate code for custom messages.

## Master

The package include ``roscore``  that is a swift implementation of the ros master. The master advertises its presense with Bonjour on Apple devices and with Avahi on Linux devices. A client initialization of Ros will search for the master unless the url is specified with remapping ``ros = Ros(name: "phone", remappings: ["__master":"http://10.0.1.23:11311"])``  or with the environment variable ``ROS_MASTER_URI``

For service discovery in iOS14 Client app, add NSBonjourServices to your Info.plist

```
<key>NSBonjourServices</key>
<array>
  <string>_ros._tcp</string>
</array>
```


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
```
The following code snippet shows how to model a simple listener

```swift
import RosSwift
import StdMsgs
import msgs

let ros = Ros(argv: &CommandLine.arguments, name: "listener")
let node = ros.createNode()
let imu = node.subscribe(topic: "/imu") { (msg: sensor_msgs.Imu) in
    print("accel: [\(msg.linear_acceleration)]")
}
n.spinThread()
```
