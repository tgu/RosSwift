import rpcobject

extension Ros {

    @available(macOS 10.15, iOS 13, tvOS 13, watchOS 6, *)
    func getTopics(callerId: String) async throws -> [TopicInfo] {
        let args = XmlRpcValue(strings: callerId, "")
        let topics = try await master.execute(method: "getPublishedTopics", request: args)
        return topics.map {
            TopicInfo(name: $0[0].string, dataType: $0[1].string )
        }
    }


}
