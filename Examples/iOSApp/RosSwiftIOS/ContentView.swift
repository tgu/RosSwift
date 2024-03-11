import SwiftUI
import RosSwift

struct ContentView: View {
    @Environment(NodeWrapper.self) var ros: NodeWrapper
    @State private var master: String = "172.16.1.158"
    @State private var message: String = ""
    @State private var subscriber: Subscriber?

    func callback(msg: String) {
        message = msg
    }

    var body: some View {
        Form {
            Section(header: Label("Master", systemImage: "network").bold() ) {
                HStack {
                    TextField(
                        "master IP:",
                        text: $master
                    ).autocorrectionDisabled(true)

                    if ros.isRunning {
                        Button("disconnect") {
                            subscriber = nil
                            ros.node = nil
                        }
                    } else {
                        Button("connect") {
                            ros.connect("http://\(master):11311")
                            if let node = ros.node {
                                subscriber = node.subscribe(topic: "/chatter", queueSize: 1, callback: callback)
                            }
                        }
                    }
                }
            }

            if ros.isRunning {
                Section(header: Label("Topics", systemImage: "tree").bold()) {
                    Text("message: \(message)")
                }
            }

        }
        .padding()
    }
}

#Preview {
    ContentView().environment(NodeWrapper())
}
