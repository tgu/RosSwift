// swift-tools-version:4.1
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

#if os(Linux)
let msgDep: [Target.Dependency] = ["OpenSSL"]
#else
let msgDep: [Target.Dependency] = []
#endif

let package = Package(
    name: "RosSwift",
    products: [
        .library(name: "RosSwift", targets: ["RosSwift"]),
        .executable(name: "publisher", targets: ["publisher"]),
        .executable(name: "listener", targets: ["listener"]),
        .executable(name: "msgbuilder", targets: ["msgbuilder"]),
        .library(name: "geometry_msgs", targets: ["geometry_msgs"]),
        .library(name: "StdMsgs", targets: ["StdMsgs"]),
        .library(name: "RosTime", targets: ["RosTime"]),
    ],
    dependencies: [
        .package(url: "https://github.com/apple/swift-nio.git", from: "1.9.5"),
        .package(url: "https://github.com/tgu/BinaryCoder.git", from: "1.0.1"),
        .package(url: "https://github.com/IBM-Swift/HeliumLogger.git", from: "1.7.3"),
        .package(url: "https://github.com/attaswift/Deque.git", from: "3.1.1"),
    ],
    targets: [
        .target( name: "RosSwift",
                 dependencies: ["StdMsgs","RosTime","BinaryCoder","NIO","HeliumLogger","Deque"],
                 path: "Sources/rosswift"),
        .target( name: "publisher",
                 dependencies: ["RosSwift","geometry_msgs","StdMsgs"]),
        .target( name: "listener",
                 dependencies: ["RosSwift","geometry_msgs","StdMsgs"]),
        .target( name: "msgbuilder",
                 dependencies: msgDep),
        .target( name: "geometry_msgs",
                 dependencies: ["StdMsgs"],
                 path: "Sources/msgs/geometry_msgs"),
        .target( name: "StdMsgs",
                 dependencies: ["RosTime"]),
        .target( name: "RosTime",
                 dependencies: ["BinaryCoder"]),
        .testTarget( name: "rosswiftTests",
                     dependencies: ["RosSwift","StdMsgs","BinaryCoder"]),
        ]
)


#if os(Linux)
package.dependencies.append(
    .package(url: "https://github.com/IBM-Swift/OpenSSL.git", from: "1.0.1"))
#endif
