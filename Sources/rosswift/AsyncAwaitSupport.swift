#if compiler(>=5.5.2) && canImport(_Concurrency)


import rpcobject

extension Master {

    @available(macOS 10.15, iOS 13, tvOS 13, watchOS 6, *)
    func execute(method: String, request: XmlRpcValue) async throws -> XmlRpcValue {
        return try await execute(method: method, request: request, host: masterHost, port: masterPort).get()
    }
    
}

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

#endif
