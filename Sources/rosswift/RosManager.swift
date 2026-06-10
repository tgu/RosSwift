//
//  RosManager.swift
//  RosSwift
//
//  Internal lifecycle protocol for the sub-managers owned by `Ros`
//  (XMLRPCManager, TopicManager, ServiceManager, ConnectionManager).
//
//  These managers are internal components of the `Ros` service rather than
//  independent `ServiceLifecycle.Service`s: they have construction-time
//  dependencies on each other and are brought up lazily when the first
//  `NodeHandle` is created. This protocol gives them a uniform lifecycle
//  surface so `Ros.start()` / `shutdown()` / `shutdownAsync()` can drive them
//  consistently in a fixed, documented order.
//

protocol RosManager: Sendable {
    /// Begin operation. Idempotent — calling more than once is a no-op.
    func start() async

    /// Synchronous teardown. Safe to call from `deinit` and non-async paths.
    func shutdown()

    /// Async teardown that awaits any in-flight master calls (e.g. the
    /// `unregister*` round-trips) before returning. Defaults to the
    /// synchronous `shutdown()` for managers that have no network drain.
    func shutdownAsync() async
}

extension RosManager {
    func shutdownAsync() async { shutdown() }
}
