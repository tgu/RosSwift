// swift-tools-version:5.6

import PackageDescription

#if os(Linux)
let msgDep: [Target.Dependency] = ["OpenSSL","StdMsgs","msgbuilderLib"]
#else
let msgDep: [Target.Dependency] = ["StdMsgs","msgbuilderLib"]
#endif

let swiftAtomics: PackageDescription.Target.Dependency = .product(name: "Atomics", package: "swift-atomics")

let package = Package(
    name: "RosSwift",
    platforms: [.macOS(.v10_14), .iOS(.v10), .tvOS(.v10), .watchOS(.v3)],
    products: [
        .library(name: "RosSwift", targets: ["RosSwift"]),
        .executable(name: "publisher", targets: ["publisher"]),
        .executable(name: "listener", targets: ["listener"]),
        .executable(name: "msgbuilder", targets: ["msgbuilder"]),
    ],
    dependencies: [
        .package(url: "https://github.com/apple/swift-nio.git", from: "2.41.1"),
        .package(url: "https://github.com/tgu/BinaryCoder.git", from: "1.1.0"),
        .package(url: "https://github.com/IBM-Swift/HeliumLogger.git", from: "1.9.200"),
        .package(url: "https://github.com/apple/swift-collections.git", from: "1.0.3"),
        .package(url: "https://github.com/apple/swift-nio-extras.git", from: "1.13.0"),
        .package(url: "https://github.com/apple/swift-log.git", from: "1.4.4"),
        .package(url: "https://github.com/apple/swift-atomics.git", from: "1.0.2"),
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
                                .product(name: "DequeModule", package: "swift-collections"),
                                "rpcobject"],
                 path: "Sources/rosswift"),
        .target( name: "msgs",
                 dependencies: ["StdMsgs","RosTime"]),
        .target( name: "StdMsgs",
                 dependencies: ["RosTime"]),
        .target( name: "RosTime",
                 dependencies: ["BinaryCoder",swiftAtomics]),
        .target( name: "rpcobject",
                 dependencies: []),
        .testTarget( name: "rosswiftTests",
                     dependencies: ["RosSwift",
                                    "BinaryCoder",
                                    "rpcobject",
                                    "rosmaster",
                                    "RosNetwork",
                                    swiftAtomics]),
        .testTarget( name: "msgBuilderTests",
                     dependencies: ["msgbuilderLib"]),
        .target(name: "RosNetwork", dependencies: [
            .product(name: "Logging", package: "swift-log"),
            .product(name: "NIOCore", package: "swift-nio")
        ]),
        .target(name: "rosmaster", dependencies: [
            "rpcclient",
            .product(name: "NIOHTTP1", package: "swift-nio"),
            .product(name: "Logging", package: "swift-log")]),
        .target(name: "rpcclient", dependencies: [
            .product(name: "NIO", package: "swift-nio"),
            swiftAtomics,
            .product(name: "Logging", package: "swift-log"),
            .target(name: "rpcobject")]),
        .executableTarget( name: "publisher",
                 dependencies: ["RosSwift"],
                 exclude: ["custom_msgs/srv/AddTwoInts.srv"]),
        .executableTarget( name: "listener",
                 dependencies: ["RosSwift"]),
        .executableTarget( name: "msgbuilder",
                 dependencies: msgDep),
        .executableTarget(name: "roscore", dependencies: [
            "rosmaster", "RosNetwork",
            .product(name: "Logging", package: "swift-log")]),
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
package.dependencies.append(.package(url: "https://github.com/Bouke/NetService.git", from: "0.7.0"))
package.targets.first(where: { $0.name == "rosmaster" })!.dependencies.append("NetService")
package.targets.first(where: { $0.name == "RosSwift" })!.dependencies.append("NetService")

#else
package.targets.append(
    .target( name: "msgbuilderLib",
             dependencies: ["StdMsgs",
                            "msgs"]))
#endif
