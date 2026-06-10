import SwiftUI
import RosSwift

@Observable class NodeWrapper {
    var node: NodeHandle?

    var isRunning: Bool {
        node?.isOK ?? false
    }

    var task: Task<Void, Never>?

    func connect(_ master: String) async throws {
        let ros = try Ros.init(name: "iPhone", remappings: ["__master":master])
        node = await ros.createNode()

        task = Task.detached(priority: .background) { [weak self] in
            await self?.node?.spinThread()
        }
    }

    func disconnect() {
        if isRunning {
            task?.cancel()
            task = nil
            node = nil
        }
    }

    deinit {
        node = nil
    }
}

@main
struct RosSwiftIOSApp: App {
    var ros = NodeWrapper()

    init() {
        // Route RosSwift logs to Apple unified logging (visible in Console.app).
        RosLogging.bootstrap(subsystem: "org.ros.RosSwiftIOS")
    }

    var body: some Scene {
        WindowGroup {
            ContentView().environment(ros)
        }
    }
}
