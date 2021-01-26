// swift-tools-version:5.2

import PackageDescription

#if os(Linux)
let msgDep: [Target.Dependency] = ["OpenSSL","StdMsgs","msgbuilderLib"]
#else
let msgDep: [Target.Dependency] = ["StdMsgs","msgbuilderLib"]
#endif

let package = Package(
    name: "RosSwift",
    platforms: [.macOS(.v10_14), .iOS(.v10), .tvOS(.v10), .watchOS(.v3)],
    products: [
        .library(name: "RosSwift", targets: ["RosSwift"]),
        .executable(name: "publisher", targets: ["publisher"]),
        .executable(name: "listener", targets: ["listener"]),
        .executable(name: "msgbuilder", targets: ["msgbuilder"]),
        .library(name: "msgs", targets: ["msgs"]),
        .library(name: "StdMsgs", targets: ["StdMsgs"]),
        .library(name: "RosTime", targets: ["RosTime"]),
        .library(name: "RosMaster", targets: ["rosmaster"]),
        .library(name: "RosNetwork", targets: ["RosNetwork"]),
    ],
    dependencies: [
        .package(url: "https://github.com/apple/swift-nio.git", from: "2.25.0"),
        .package(url: "https://github.com/tgu/BinaryCoder.git", from: "1.1.0"),
        .package(url: "https://github.com/IBM-Swift/HeliumLogger.git", from: "1.9.0"),
        .package(url: "https://github.com/tgu/Deque.git", from: "3.1.2"),
        .package(url: "https://github.com/apple/swift-nio-extras.git", from: "1.7.0"),
        .package(url: "https://github.com/apple/swift-log.git", from: "1.4.0"),
    ],
    targets: [
        .target( name: "RosSwift",
                 dependencies: ["StdMsgs",
                                "msgs",
                                "RosTime",
                                "BinaryCoder",
                                .product(name: "NIO", package: "swift-nio"),
                                .product(name: "NIOHTTP1", package: "swift-nio"),
                                .product(name: "NIOExtras", package: "swift-nio-extras"),
                                "RosNetwork",
                                "HeliumLogger",
                                "Deque",
                                "rpcobject"],
                 path: "Sources/rosswift"),
        .target( name: "publisher",
                 dependencies: ["RosSwift","msgs","StdMsgs"]),
        .target( name: "listener",
                 dependencies: ["RosSwift","msgs","StdMsgs"]),
        .target( name: "msgbuilder",
                 dependencies: msgDep),
        .target( name: "msgs",
                 dependencies: ["StdMsgs","RosTime"]),
        .target( name: "StdMsgs",
                 dependencies: ["RosTime"]),
        .target( name: "RosTime",
                 dependencies: ["BinaryCoder"]),
        .target( name: "rpcobject",
                 dependencies: []),
        .testTarget( name: "rosswiftTests",
                     dependencies: ["RosSwift",
                                    "StdMsgs",
                                    "BinaryCoder",
                                    "rpcobject",
                                    "msgs",
                                    .product(name: "NIOConcurrencyHelpers", package: "swift-nio")]),
        .testTarget( name: "msgBuilderTests",
                     dependencies: ["msgbuilderLib"]),
        .target(name: "RosNetwork", dependencies: [
            .product(name: "Logging", package: "swift-log"),
            .product(name: "NIO", package: "swift-nio")
        ]),
        .target(name: "roscore", dependencies: [
            "rosmaster", "RosNetwork",
            .product(name: "Logging", package: "swift-log")]),
        .target(name: "rosmaster", dependencies: [
            "rpcclient",
            .product(name: "NIOHTTP1", package: "swift-nio"),
            .product(name: "Logging", package: "swift-log")]),
        .target(name: "rpcclient", dependencies: [
            .product(name: "NIO", package: "swift-nio"),
            .product(name: "NIOConcurrencyHelpers", package: "swift-nio"),
            .product(name: "Logging", package: "swift-log"),
            .target(name: "rpcobject")]),
        .target(name: "rosparam", dependencies: ["rpcclient","RosNetwork"]),
        .testTarget(name: "rosmasterTests", dependencies: ["roscore"]),

    ]
)


#if os(Linux)
package.dependencies.append(
    .package(url: "https://github.com/IBM-Swift/OpenSSL.git", from: "2.2.2"))
package.targets.append(
    .target( name: "msgbuilderLib",
             dependencies: ["StdMsgs","OpenSSL",
                            "msgs"]))
#else
package.targets.append(
    .target( name: "msgbuilderLib",
             dependencies: ["StdMsgs",
                            "msgs"]))
#endif
