import SwiftUI
import RosSwift

@Observable class NodeWrapper {
    var node: NodeHandle?

    var isRunning: Bool {
        node?.ok ?? false
    }

    func connect(_ master: String) {
        let ros = Ros.init(name: "iPhone", remappings: ["__master":master])
        node = ros.createNode()

        Task.detached(priority: .background) { [weak self] in
            self?.node?.spinThread()
        }
    }

    deinit {
        node = nil
    }
}

@main
struct RosSwiftIOSApp: App {
    var ros = NodeWrapper()
    var body: some Scene {
        WindowGroup {
            ContentView().environment(ros)
        }
    }
}
