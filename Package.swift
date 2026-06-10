// swift-tools-version:6.2

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
    .executable(name: "roscore", targets: ["roscore"]),
    .library(name: "RosSwift", targets: ["RosSwift"]),
    .library(name: "msgs", targets: ["msgs"]),
    .library(name: "StdMsgs", targets: ["StdMsgs"]),
    .library(name: "RosTime", targets: ["RosTime"]),
    .library(name: "rosmaster", targets: ["rosmaster"]),
    .library(name: "RosNetwork", targets: ["RosNetwork"]),
]

var targets: [Target] = [
    .target( name: "RosSwift",
             dependencies: [.byName(name: "StdMsgs"),
                            .byName(name: "msgs"),
                            .byName(name: "RosTime"),
                            .byName(name: "BinaryCoder"),
                            .product(name: "NIO", package: "swift-nio"),
                            .product(name: "NIOHTTP1", package: "swift-nio"),
                            .product(name: "NIOExtras", package: "swift-nio-extras"),
                            .byName(name: "RosNetwork"),
                            .product(name: "Logging", package: "swift-log"),
                            .product(name: "DequeModule", package: "swift-collections"),
                            .product(name: "AsyncAlgorithms", package: "swift-async-algorithms"),
                            .product(name: "ServiceLifecycle", package: "swift-service-lifecycle"),
                            .byName(name: "rpcobject")],
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
    .package(url: "https://github.com/apple/swift-nio.git", from: "2.91.0"),
    .package(url: "https://github.com/tgu/BinaryCoder.git", from: "1.1.0"),
    .package(url: "https://github.com/apple/swift-collections.git", from: "1.3.0"),
    .package(url: "https://github.com/apple/swift-nio-extras.git", from: "1.31.2"),
    .package(url: "https://github.com/apple/swift-log.git", from: "1.8.0"),
    .package(url: "https://github.com/apple/swift-atomics.git", from: "1.3.0"),
    .package(url: "https://github.com/apple/swift-async-algorithms.git", from: "1.1.0"),
    .package(url: "https://github.com/swift-server/swift-service-lifecycle.git", from: "2.9.1"),
]


#if os(Linux) || os(macOS)
products.append(.executable(name: "publisher", targets: ["publisher"]))
products.append(.executable(name: "listener", targets: ["listener"]))
products.append(.executable(name: "msgbuilder", targets: ["msgbuilder"]))

targets.append(.executableTarget( name: "listener", dependencies: [
    "RosSwift",
    .product(name: "ServiceLifecycle", package: "swift-service-lifecycle"),
    .product(name: "Logging", package: "swift-log")]))
targets.append(.executableTarget( name: "publisher",
                                      dependencies: [
                                        "RosSwift",
                                        .product(name: "ServiceLifecycle", package: "swift-service-lifecycle"),
                                        .product(name: "Logging", package: "swift-log")],
                                      exclude: ["custom_msgs/srv/AddTwoInts.srv"]))
targets.append(.executableTarget( name: "msgbuilder", dependencies: msgDep))
targets.append(.testTarget( name: "msgBuilderTests", dependencies: ["msgbuilderLib"]))
targets.append(.executableTarget(name: "roscore", dependencies: [
    "rosmaster", "RosNetwork",
    .product(name: "ServiceLifecycle", package: "swift-service-lifecycle"),
    .product(name: "Logging", package: "swift-log")]))
targets.append(.target(name: "rosmaster", dependencies: [
    "rpcclient",
    .product(name: "NIOHTTP1", package: "swift-nio"),
    .product(name: "NIOExtras", package: "swift-nio-extras"),
    .product(name: "ServiceLifecycle", package: "swift-service-lifecycle"),
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
                                               .product(name: "AsyncAlgorithms", package: "swift-async-algorithms"),
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
    platforms: [.macOS("15.4"), .iOS("18.4"), .tvOS("18.4")],
    products: products,
    dependencies: dependencies,
    targets: targets
)


