//
//  MasterService.swift
//  rosmaster
//
//  ServiceLifecycle.Service conformance for Master so it can participate
//  in a structured-concurrency application lifecycle via ServiceGroup
//  alongside other services (e.g. a Ros node).
//

import ServiceLifecycle
import Synchronization

extension Master: Service {
    /// Run the master's lifecycle as a structured-concurrency Service.
    ///
    /// Binds the listener via `start()`, then parks until the owning
    /// `ServiceGroup` signals graceful shutdown (or the surrounding task is
    /// cancelled). On shutdown, calls `stop()` and awaits its completion so
    /// in-flight requests get a chance to drain.
    ///
    /// Usage:
    /// ```swift
    /// let master = rosmaster.Master(host: host, port: 11311, advertise: false)
    /// let ros = try Ros(name: "my_node", master: host)
    /// let group = ServiceGroup(
    ///     services: [master, ros],
    ///     gracefulShutdownSignals: [.sigterm, .sigint],
    ///     logger: logger
    /// )
    /// try await group.run()
    /// ```
    public func run() async throws {
        _ = try await start().get()

        // Park until graceful shutdown is signalled. We suspend on a
        // continuation and resume it from the onGracefulShutdown callback.
        // The `shutdownRequested` flag closes the race where shutdown fires
        // before the continuation has been stored.
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

        _ = try? await stop().get()
    }
}
