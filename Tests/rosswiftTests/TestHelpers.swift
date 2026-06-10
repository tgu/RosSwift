//
//  TestHelpers.swift
//  rosswiftTests
//
//  Helpers for clean Ros teardown across tests.
//

@testable import RosSwift

/// Runs `body` with a freshly constructed `Ros`, then awaits `Ros.shutdownAsync()`
/// before returning so the master receives `unregister*` calls *before* it gets
/// torn down by the surrounding test fixture's `deinit`. Pass `.noSigintHandler`
/// by default so tests don't fight over signal handlers.
///
/// Usage:
/// ```swift
/// @Test func intraprocess() async throws {
///     try await withRos(master: host) { ros in
///         let n = await ros.createNode()
///         // ... test body ...
///     }
/// }
/// ```
func withRos<T: Sendable>(
    name: String = #function,
    master: String,
    port: Int = 11311,
    namespace: String = "",
    remappings: StringStringMap = [:],
    options: Ros.InitOption = [],
    body: (Ros) async throws -> T
) async throws -> T {
    // `#function` yields e.g. "publisherCallback()"; ROS node names cannot
    // contain parens, so strip everything from the first '(' onwards.
    var sanitizedName = name
    if let paren = sanitizedName.firstIndex(of: "(") {
        sanitizedName = String(sanitizedName[sanitizedName.startIndex..<paren])
    }
    let mergedOptions = options.union([.noSigintHandler])
    let ros = try Ros(name: sanitizedName, master: master, port: port, options: mergedOptions)
    do {
        let result = try await body(ros)
        await ros.shutdownAsync()
        return result
    } catch {
        await ros.shutdownAsync()
        throw error
    }
}

extension Ros {
    /// Test-only convenience that wraps the existing failable
    /// `init(name:master:port:)` to accept named-options.
    convenience init(name: String, master: String, port: Int = 11311, options: InitOption) throws {
        try self.init(name: name,
                      remappings: ["__master": "http://\(master):\(port)"],
                      unhandledArgs: [],
                      options: options)
    }
}
