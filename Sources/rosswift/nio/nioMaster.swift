//
//  Master.swift
//  swiftros
//
//  Created by Thomas Gustafsson on 2018-03-19.
//

import Foundation
import NIO
import XMLRPCSerialization

let thread_group = MultiThreadedEventLoopGroup(numberOfThreads: 1)

struct TopicInfo {
    let name: String
    let dataType: String
}

let XMLRPC_VERSION = "XMLRPC++ 0.7"

struct XMLRPCClient {
    static let REQUEST_BEGIN = "<?xml version=\"1.0\"?>\r\n<methodCall>\r\n<methodName>"
    static let REQUEST_END_METHODNAME = "</methodName>"
    static let PARAMS_TAG = "<params>"
    static let PARAMS_ETAG = "</params>"
    static let PARAM_TAG = "<param>"
    static let PARAM_ETAG =  "</param>"
    static let REQUEST_END = "</methodCall>\r\n"
    static let METHODRESPONSE_TAG = "<methodResponse>"
    static let FAULT_TAG = "<fault>"

}


    class Master {
        public static var shared = Master(group: thread_group)


        enum Response {
            case xmlrpc(XMLRPCResponse)
            case error(String)
        }

//        var g_uri = "http://localhost:11311"
        var g_host = "127.0.0.1"
        var g_port: UInt16 = 11311
        var g_retry_timeout = 0.0


        public func initialize(remappings: M_string)
        {
            var g_uri = remappings["__master"]
            if g_uri == nil {
                var master_uri_env = ProcessInfo.processInfo.environment["ROS_MASTER_URI"]
                if master_uri_env == nil {
                    if amIBeingDebugged() {
                        master_uri_env = "http://localhost:11311"
                    } else {
                        fatalError( "ROS_MASTER_URI is not defined in the environment. Either " +
                            "type the following or (preferrably) add this to your " +
                            "~/.bashrc file in order set up your " +
                            "local machine as a ROS master:\n\n" +
                            "export ROS_MASTER_URI=http://localhost:11311\n\n" +
                            "then, type 'roscore' in another shell to actually launch " +
                            "the master program.")
                    }
                }
                g_uri = master_uri_env
            }

            // Split URI into
            if (!Ros.network.splitURI(uri: g_uri!, host: &g_host, port: &g_port)) {
                fatalError( "Couldn't parse the master URI [\(g_uri!)] into a host:port pair.")
            }
            ROS_DEBUG("master on host: \(g_host), port: \(g_port)")
        }

        final class XmlRpcMessageDelimiterCodec: ByteToMessageDecoder {
            public typealias InboundIn = ByteBuffer
            public typealias InboundOut = ByteBuffer

            public var cumulationBuffer: ByteBuffer?

            public func decode(ctx: ChannelHandlerContext, buffer: inout ByteBuffer) throws -> DecodingState {

                guard let header = buffer.getString(at: buffer.readerIndex, length: buffer.readableBytes) else {
                    return .needMoreData
                }

                guard let range = header.lowercased().range(of: "content-length: ")  else {
                    return .needMoreData
                }

                let content = String(header[range.upperBound..<header.endIndex])
                guard let index = content.index(of: "\r\n") else {
                    return .needMoreData
                }

                guard let length = Int(content[content.startIndex..<index]) else {
                    return .needMoreData
                }

                guard let ind2 = content.index(of: "<") else {
                    ROS_DEBUG("Malformed header")
                    return .needMoreData
                }

                if content.count >= length {
                    ctx.fireChannelRead(self.wrapInboundOut(buffer.readSlice(length: buffer.readableBytes)!))
                    return .continue
                }

                return .needMoreData
            }

            func decodeLast(ctx: ChannelHandlerContext, buffer: inout ByteBuffer) throws -> DecodingState {
                /// set cumulationBuffer to avoid to be called again with the same data
                /// https://github.com/apple/swift-nio/issues/108
                ///
                self.cumulationBuffer = nil
                return .continue
            }
        }


        class XmlRpcHandler: ChannelInboundHandler {
            public typealias InboundIn = ByteBuffer
            public typealias OutboundOut = ByteBuffer

            var response = XMLRPCResponse.fault(code: 1, string: "not initiated")
            weak var owner : Master!

            init(owner: Master) {
                self.owner = owner
            }


            func channelActive(ctx: ChannelHandlerContext) {
                owner.registerHandler(for: ctx.channel, handler: self)
            }


            public func channelRead(ctx: ChannelHandlerContext, data: NIOAny) {
               var buffer = self.unwrapInboundIn(data)
                if let string = buffer.readString(length: buffer.readableBytes) {
                    do {

                        guard let range = string.lowercased().range(of: "content-length: ")  else {
                            ROS_DEBUG("Header not read")
                            return
                        }

                        let content = String(string[range.upperBound..<string.endIndex])
                        guard let index = content.index(of: "\r\n") else {
                            ROS_DEBUG("Malformed header")
                            return
                        }

                        guard let _ = Int(content[content.startIndex..<index]) else {
                            ROS_DEBUG("length error")
                            return
                        }

                        guard let ind2 = content.index(of: "<") else {
                            ROS_DEBUG("Malformed header")
                            return
                        }

                        let _response = String(content[ind2..<content.endIndex])

                        if let d = _response.data(using: .utf8) {
                            response = try XMLRPCSerialization.xmlrpcResponse(from: d )
                        }
                    } catch {
                        ROS_ERROR(error.localizedDescription)
                    }
                    ctx.close(promise: nil)
                }
            }

            public func errorCaught(ctx: ChannelHandlerContext, error: Error) {
                ROS_ERROR(error.localizedDescription)

                // As we are not really interested getting notified on success or failure we just pass nil as promise to
                // reduce allocations.
                ctx.close(promise: nil)
            }
        }

        let group : EventLoopGroup
        var bootstrap : ClientBootstrap? = nil
        var handlers = [ObjectIdentifier: XmlRpcHandler]()
        var channel : Channel?

        var uri : String {
            return "/"
        }

        var host : String {
            return g_host
        }

        var port : UInt16 {
            return g_port
        }

        func registerHandler(for channel: Channel, handler: XmlRpcHandler) {
            precondition(handlers[ObjectIdentifier(channel)] == nil)
            handlers[ObjectIdentifier(channel)] = handler
        }

        func unregisterHandler(for channel: Channel) {
            precondition(handlers[ObjectIdentifier(channel)] != nil)
            handlers.removeValue(forKey: ObjectIdentifier(channel))
        }

        private init(group: EventLoopGroup) {
            self.group = group
//            let handler = XmlRpcHandler()
//            self.handler = handler
            self.bootstrap = ClientBootstrap(group: group)
                // Enable SO_REUSEADDR.
                .channelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
                .channelInitializer { channel in
                    channel.pipeline.addHandlers([XmlRpcMessageDelimiterCodec(),XmlRpcHandler(owner: self)], first: true)
            }
        }


        func validateXmlrpcResponse(method: String, response: XmlRpcValue) -> XmlRpcValue? {

            guard response.isArray else {
                ROS_DEBUG("XML-RPC call [\(method)] didn't return an array")
                return nil
            }

            var resp = response
            if resp.size() == 1 {
                resp = resp[0]
            }

            if resp.size() != 2 && resp.size() != 3 {
                ROS_DEBUG("XML-RPC call [\(method)] didn't return a 2 or 3-element array")
                return nil
            }
            guard case .int(let status_code) = resp[0].value else {
                ROS_DEBUG("XML-RPC call [\(method)] didn't return a int as the 1st element")
                return nil
            }
            guard case .string(let status_string) = resp[1].value else {
                ROS_DEBUG("XML-RPC call [\(method)] didn't return a string as the 2nd element")
                return nil
            }
            if status_code != 1 {
                ROS_DEBUG("XML-RPC call [\(method)] returned an error (\(status_code): [\(status_string)]")
                return nil
            }
            if resp.size() > 2 {
                if case .string(let uri) = resp[2].value {
                    return XmlRpcValue(anyArray: [uri])
                } else {
                    return resp[2]
                }
            }
            return XmlRpcValue(anyArray: [])
        }

        func generateHeader(body: String, host: String, port: UInt16) -> String
        {
            let header = "POST "
                + uri
                + " HTTP/1.1\r\nUser-Agent: "
                + XMLRPC_VERSION
                + "\r\nHost: "
                + host
                + ":\(port)\r\n"
                + "Content-Type: text/xml\r\nContent-length: "
                + "\(body.lengthOfBytes(using: .utf8))\r\n\r\n"
            return header
        }

        func generateRequest(methodName: String, params: XmlRpcValue, host: String, port: UInt16) -> String
        {
            var body = XMLRPCClient.REQUEST_BEGIN + methodName + XMLRPCClient.REQUEST_END_METHODNAME

            // If params is an array, each element is a separate parameter
            if params.valid() {
                body += XMLRPCClient.PARAMS_TAG
                if case .array(let array) = params.value {
                    for a in array {
                        body += XMLRPCClient.PARAM_TAG
                        body += a.toXml()
                        body += XMLRPCClient.PARAM_ETAG
                    }
                }
                else
                {
                    body += XMLRPCClient.PARAM_TAG
                    body += params.toXml()
                    body += XMLRPCClient.PARAM_ETAG
                }

                body += XMLRPCClient.PARAMS_ETAG
            }
            body += XMLRPCClient.REQUEST_END

            let header = generateHeader(body: body, host: host, port: port)
            let _request = header + body
            return _request
        }

        private func validate(response: XMLRPCResponse, for method: String) -> XmlRpcValue {
            switch response {
            case .fault(let code, let string):
                ROS_DEBUG("\(code) - \(string)")
            case .response(let any):
                let r = XmlRpcValue(any: any)
                guard let payload = self.validateXmlrpcResponse(method: method, response: r) else {
                    return XmlRpcValue()
                }
                return payload
            }
            return XmlRpcValue()
        }

        func execute(method: String, request: XmlRpcValue) -> EventLoopFuture<XmlRpcValue> {
            return execute(method: method, request: request, host: host, port: g_port)

        }

        enum MasterError: Error {
            case invalidResponse(String)
        }

        func execute(method: String, request: XmlRpcValue, host: String, port: UInt16) -> EventLoopFuture<XmlRpcValue> {
            let xml = generateRequest(methodName: method, params: request, host: host, port: port)

            let eventLoop = group.next()
            let promise: EventLoopPromise<XmlRpcValue> = eventLoop.newPromise()

            ROS_DEBUG("trying to connect to \(host):\(port) for method \(method)")

            bootstrap?.connect(host: host, port: Int(port)).map { (channel) -> Void in
                var buffer = channel.allocator.buffer(capacity: xml.utf8.count)
                buffer.write(string: xml)
                _ = channel.writeAndFlush(buffer).whenFailure { (error) in
                    ROS_ERROR("write failed to \(channel.remoteAddress!) [\(error)]")
                }
                channel.closeFuture.whenComplete {

                    guard let handler = self.handlers[ObjectIdentifier(channel)] else {
                        fatalError()
                        return
                    }

                    let result = self.validate(response: handler.response, for: method)

                    self.unregisterHandler(for: channel)
                    if result.valid() {
                        promise.succeed(result: result)
                    } else {
                        promise.fail(error: MasterError.invalidResponse(result.description))
                    }
                }
                }.whenFailure { (error) in
                    ROS_ERROR(error.localizedDescription)
            }
            return promise.futureResult
            

        }


        func getTopics() -> [TopicInfo]? {
            let args = XmlRpcValue(array: [XmlRpcValue(str: Ros.this_node.getName()),XmlRpcValue(str: "")])
            do {
                let payload = try execute(method: "getPublishedTopics", request: args).wait()

                var topics = [TopicInfo]()
                for i in 0..<payload.size() {
                    topics.append(TopicInfo(name: payload[i][0].string, dataType: payload[i][1].string ) )
                }

                return topics
            } catch {
                ROS_ERROR("getPublishedTopics failed: \(error)")
            }
            return nil
        }


    }

