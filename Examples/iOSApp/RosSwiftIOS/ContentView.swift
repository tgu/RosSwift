import SwiftUI
import RosSwift

extension TopicInfo: @retroactive Equatable {}
extension TopicInfo: @retroactive Identifiable, @retroactive Hashable {
    public static func == (lhs: TopicInfo, rhs: TopicInfo) -> Bool {
        lhs.id == rhs.id
    }
    
    public func hash(into hasher: inout Hasher) {
        hasher.combine(id)
    }
    
    public var id: String {
        name
    }
}

struct ContentView: View {
    @Environment(NodeWrapper.self) var ros: NodeWrapper
    @State private var master: String = "127.0.0.1"
    @State private var message: String = ""
    @State private var topics: [TopicInfo] = []
    @State private var subscriber: Subscriber?
    @State private var selectedTopic: String?

    func callback(msg: String) async {
        message = msg
    }

    var body: some View {
        Form {
            Section(header: Label("Master", systemImage: "network").bold() ) {
                HStack {
                    TextField(
                        "master IP:",
                        text: $master
                    )
                    .autocorrectionDisabled(true)
                    .onSubmit {
                        Task {
                            try? await ros.connect("http://\(master):11311")
                            topics = (try? await ros.node?.ros.getTopics()) ?? []
                        }
                    }

                    if ros.isRunning {
                        Button("disconnect") {
                            subscriber?.unsubscribe()
                            ros.disconnect()
                        }
                    }
                }
            }

            if ros.isRunning {
                Section(header: Label("Topics", systemImage: "tree").bold()) {
                    List(topics, selection: $selectedTopic) { (topic: TopicInfo) in
                        Text("\(topic.name): \(topic.dataType)")
                    }.refreshable {
                        topics = (try? await ros.node?.ros.getTopics()) ?? []
                    }
                }

                Section(header: Label("Chatter", systemImage: "tree").bold()) {
                    Text(message)
                }.task {
                    subscriber = await ros.node?.subscribe(topic: "/chatter", queueSize: 1) { msg in
                        Task {
                            await MainActor.run {
                                message = msg
                            }
                        }
                    }
                }
            }

        }.padding()
    }
}

#Preview {
    ContentView().environment(NodeWrapper())
}
