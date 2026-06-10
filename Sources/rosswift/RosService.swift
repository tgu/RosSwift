//
//  RosService.swift
//  RosSwift
//
//  ServiceLifecycle.Service conformance for Ros so it can participate in
//  structured-concurrency-driven application lifecycles via ServiceGroup.
//

import ServiceLifecycle
import Synchronization

extension Ros: ServiceLifecycle.Service {
    /// Run the node's lifecycle as a structured-concurrency Service.
    ///
    /// Starts the node if it hasn't been started yet, then blocks until
    /// graceful shutdown is signaled by the owning `ServiceGroup` (or task
    /// cancellation). On shutdown, drains in-flight master calls via
    /// `shutdownAsync()` so the master receives a clean teardown.
    ///
    /// Usage:
    /// ```swift
    /// let ros = try Ros(name: "my_node")
    /// let group = ServiceGroup(
    ///     services: [ros],
    ///     gracefulShutdownSignals: [.sigterm, .sigint],
    ///     logger: logger
    /// )
    /// try await group.run()
    /// ```
    public func run() async throws {
        if !isStarted {
            await start()
        }

        // Park until graceful shutdown is signalled by the ServiceGroup. We
        // suspend on a continuation and resume it from the onGracefulShutdown
        // callback. The `shutdownRequested` flag closes the race where
        // shutdown fires before the continuation has been stored.
        struct ParkState {
            var continuation: CheckedContinuation<Void, Never>?
            var shutdownRequested = false
        }
        let box = Mutex(ParkState())

        await withGracefulShutdownHandler {
            await withCheckedContinuation { (cont: CheckedContinuation<Void, Never>) in
                let resumeNow: Bool = box.withLock { state in
                    if state.shutdownRequested { return true }
                    state.continuation = cont
                    return false
                }
                if resumeNow { cont.resume() }
            }
        } onGracefulShutdown: {
            box.withLock { state in
                state.shutdownRequested = true
                state.continuation?.resume()
                state.continuation = nil
            }
        }

        await shutdownAsync()
    }
}
