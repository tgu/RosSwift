// swift-tools-version:5.9

import PackageDescription

#if os(Linux)
let msgDep: [Target.Dependency] = [
//    "OpenSSL",
    "StdMsgs","msgbuilderLib"]
#else
let msgDep: [Target.Dependency] = ["StdMsgs","msgbuilderLib"]
#endif

let swiftAtomics: Target.Dependency = .product(name: "Atomics", package: "swift-atomics")

var products: [Product] = [
    .library(name: "RosSwift", targets: ["RosSwift"]),
    .library(name: "msgs", targets: ["msgs"]),
    .library(name: "StdMsgs", targets: ["StdMsgs"]),
    .library(name: "RosTime", targets: ["RosTime"]),
]

var targets: [Target] = [
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
    .target( name: "msgs", dependencies: ["StdMsgs","RosTime"]),
    .target( name: "StdMsgs", dependencies: ["RosTime"]),
    .target( name: "RosTime", dependencies: ["BinaryCoder",swiftAtomics]),
    .target( name: "rpcobject", dependencies: []),
    .target(name: "RosNetwork", dependencies: [
        .product(name: "Logging", package: "swift-log"),
        .product(name: "NIOCore", package: "swift-nio")
    ]),
]

var dependencies: [Package.Dependency] = [
    .package(url: "https://github.com/apple/swift-nio.git", from: "2.41.1"),
    .package(url: "https://github.com/tgu/BinaryCoder.git", from: "1.1.0"),
    .package(url: "https://github.com/IBM-Swift/HeliumLogger.git", from: "1.9.200"),
    .package(url: "https://github.com/apple/swift-collections.git", from: "1.0.3"),
    .package(url: "https://github.com/apple/swift-nio-extras.git", from: "1.13.0"),
    .package(url: "https://github.com/apple/swift-log.git", from: "1.4.4"),
    .package(url: "https://github.com/apple/swift-atomics.git", from: "1.0.2"),
]


#if os(Linux) || os(macOS)
products.append(.executable(name: "publisher", targets: ["publisher"]))
products.append(.executable(name: "listener", targets: ["listener"]))
products.append(.executable(name: "msgbuilder", targets: ["msgbuilder"]))

targets.append(.executableTarget( name: "listener", dependencies: ["RosSwift"]))
targets.append(.executableTarget( name: "publisher",
                                      dependencies: ["RosSwift"],
                                      exclude: ["custom_msgs/srv/AddTwoInts.srv"]))
targets.append(.executableTarget( name: "msgbuilder", dependencies: msgDep))
targets.append(.testTarget( name: "msgBuilderTests", dependencies: ["msgbuilderLib"]))
targets.append(.executableTarget(name: "roscore", dependencies: [
    "rosmaster", "RosNetwork",
    .product(name: "Logging", package: "swift-log")]))
targets.append(.target(name: "rosmaster", dependencies: [
    "rpcclient",
    .product(name: "NIOHTTP1", package: "swift-nio"),
    .product(name: "Logging", package: "swift-log")]))
targets.append(.testTarget(name: "rosmasterTests", dependencies: ["roscore"]))
targets.append(.target(name: "rpcclient", dependencies: [
    .product(name: "NIO", package: "swift-nio"),
    swiftAtomics,
    .product(name: "Logging", package: "swift-log"),
    .target(name: "rpcobject")]))
targets.append(.testTarget( name: "rosswiftTests",
                                dependencies: ["RosSwift",
                                               "BinaryCoder",
                                               "rpcobject",
                                               "rosmaster",
                                               "RosNetwork",
                                               swiftAtomics]))
#endif

#if os(Linux)
//dependencies.append(
//    .package(url: "https://github.com/IBM-Swift/OpenSSL.git", from: "2.2.2"))
targets.append(
    .target( name: "msgbuilderLib",
             dependencies: ["StdMsgs",
//                            "OpenSSL",
                            "msgs"]))
//dependencies.append(.package(url: "https://github.com/Bouke/NetService.git", from: "0.7.0"))
//targets.first(where: { $0.name == "rosmaster" })!.dependencies.append("NetService")
//targets.first(where: { $0.name == "RosSwift" })!.dependencies.append("NetService")
#endif

#if os(macOS)
targets.append(
    .target( name: "msgbuilderLib",
             dependencies: ["StdMsgs",
                            "msgs"]))
#endif




let package = Package(
    name: "RosSwift",
    platforms: [.macOS(.v13), .iOS(.v14), .tvOS(.v14), .watchOS(.v7), .visionOS(.v1)],
    products: products,
    dependencies: dependencies,
    targets: targets
)


