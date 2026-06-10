import Foundation
import Logging
import rosmaster
import RosNetwork
import ServiceLifecycle

@main
struct Roscore {
    static func main() async throws {
        // Same logging as the rest of the codebase: Apple unified logging on
        // Apple platforms, stdout on Linux.
        RosLogging.bootstrap()
        let logger = Logger(label: "roscore")

        let network = RosNetwork(remappings: [:])
        let master = Master(host: network.gHost, port: defaultMasterPort)

        logger.info("version \(appVersion) listening at http://\(master.host):\(master.port)/")

        // ServiceGroup owns the signal traps and orchestrates startup/graceful
        // shutdown. Master.run() binds the listener, parks until shutdown,
        // then drives ServerQuiescingHelper for an in-flight-friendly stop.
        let group = ServiceGroup(
            services: [master],
            gracefulShutdownSignals: [.sigterm, .sigint],
            logger: logger
        )
        try await group.run()
    }
}
