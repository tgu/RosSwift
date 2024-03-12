//
//  Master.swift
//  swiftros
//
//  Created by Thomas Gustafsson on 2018-03-19.
//

import Foundation
import NIO
import NIOConcurrencyHelpers
import rpcobject
import RosNetwork
#if os(Linux)
import NetService
#endif

let threadGroup = MultiThreadedEventLoopGroup(numberOfThreads: 1)

/// Topic information
///
/// The type is represented in the format "package_name/MessageName"

public struct TopicInfo {
    public let name: String
    public let dataType: String
}

let xmlrpcVersion = "XMLRPC++ 0.7"

final class XmlRpcMessageDelimiterCodec: ByteToMessageDecoder {
    typealias InboundIn = ByteBuffer
    typealias InboundOut = ByteBuffer

    var cumulationBuffer: ByteBuffer?

    func decode(context: ChannelHandlerContext, buffer: inout ByteBuffer) throws -> DecodingState {

        guard let header = buffer.getString(at: buffer.readerIndex, length: buffer.readableBytes) else {
            return .needMoreData
        }

        guard let range = header.lowercased().range(of: "content-length: ")  else {
            return .needMoreData
        }

        let content = String(header[range.upperBound..<header.endIndex])
        guard let index = content.firstIndex(of: "\r\n") else {
            return .needMoreData
        }

        guard let length = Int(content[content.startIndex..<index]) else {
            return .needMoreData
        }

        guard let _ = content.firstIndex(of: "<") else {
            ROS_DEBUG("Malformed header")
            return .needMoreData
        }

        if content.count >= length {
            context.fireChannelRead(self.wrapInboundOut(buffer.readSlice(length: buffer.readableBytes)!))
            return .continue
        }

        return .needMoreData
    }

    func decodeLast(context: ChannelHandlerContext, buffer: inout ByteBuffer, seenEOF: Bool) throws -> DecodingState {
        /// set cumulationBuffer to avoid to be called again with the same data
        /// https://github.com/apple/swift-nio/issues/108
        ///
        self.cumulationBuffer = nil
        return .continue
    }
}

final class XmlRpcHandler: ChannelInboundHandler {
    typealias InboundIn = ByteBuffer
    typealias OutboundOut = ByteBuffer

    var response = XmlRpcValue()
    weak var owner: Master!

    init(owner: Master) {
        self.owner = owner
    }

    func channelActive(context: ChannelHandlerContext) {
        owner.registerHandler(for: context.channel, handler: self)
    }

    func channelRead(context: ChannelHandlerContext, data: NIOAny) {
        var buffer = self.unwrapInboundIn(data)
        if let string = buffer.readString(length: buffer.readableBytes) {
            guard let range = string.lowercased().range(of: "content-length: ")  else {
                ROS_DEBUG("Header not read")
                return
            }

            let content = String(string[range.upperBound..<string.endIndex])
            guard let index = content.firstIndex(of: "\r\n") else {
                ROS_DEBUG("Malformed header")
                return
            }

            guard let _ = Int(content[content.startIndex..<index]) else {
                ROS_DEBUG("length error")
                return
            }

            guard let ind2 = content.firstIndex(of: "<") else {
                ROS_DEBUG("Malformed header")
                return
            }

            let resp = String(content[ind2..<content.endIndex])
            if let r = XMLRPCClient.parseResponse(xml: resp) {
                response = r
            }
            context.close(promise: nil)
        }
    }

    func errorCaught(context: ChannelHandlerContext, error: Error) {
        ROS_ERROR(error.localizedDescription)

        // As we are not really interested getting notified on success or failure we just pass nil as promise to
        // reduce allocations.
        context.close(promise: nil)
    }
}

enum XMLRPCClient {
    static let requestBegin = "<?xml version=\"1.0\"?>\r\n<methodCall>\r\n<methodName>"
    static let requestEnd = "</methodCall>\r\n"
    static let methodresponseTag = "<methodResponse>"

    static func parseResponse(xml: String) -> XmlRpcValue? {
        guard let tagIndex = xml.range(of: methodresponseTag) else {
            ROS_ERROR("Invalid response - no methodResponse. Response:\n\(xml)")
            return nil
        }

        var result = XmlRpcValue()

        var data = xml.suffix(from: tagIndex.upperBound)
        if XmlRpcUtil.nextTagIs(tag: .params, xml: &data) && XmlRpcUtil.nextTagIs(tag: .param, xml: &data) {
            if result.fromXML(xml: &data) {
                return result
            } else {
                ROS_ERROR("Invalid response value. Response: \(data)")
            }
        } else if XmlRpcUtil.nextTagIs(tag: .fault, xml: &data) {
            ROS_ERROR("Invalid response value. Response: \(data)")
        } else {
            ROS_ERROR("Invalid response - no param or fault tag. Response \(xml)")
        }

        return nil
    }

}



final class Master {

    let masterHost: String
    let masterPort: UInt16
    let group: EventLoopGroup
    var bootstrap: ClientBootstrap?
    var handlers = [ObjectIdentifier: XmlRpcHandler]()
    let lock = NIOLock()
    
    var path: String {
        return "\(masterHost):\(masterPort)"
    }

    var uri: String {
        return "/"
    }

    func registerHandler(for channel: Channel, handler: XmlRpcHandler) {
        lock.withLock {
            precondition(handlers[ObjectIdentifier(channel)] == nil)
            handlers[ObjectIdentifier(channel)] = handler
        }
    }

    func unregisterHandler(for channel: Channel) {
        lock.withLock {
            precondition(handlers[ObjectIdentifier(channel)] != nil)
            handlers.removeValue(forKey: ObjectIdentifier(channel))
        }
    }
    
    static func determineRosMasterAddress(remappings: StringStringMap) -> (host: String, port: UInt16) {
        var masterURI = remappings["__master"]
#if os(macOS) || os(iOS) || os(tvOS) || os(Linux)
        if masterURI == nil {
            var masterUriEnv = ProcessInfo.processInfo.environment["ROS_MASTER_URI"]
            if masterUriEnv == nil {
                ROS_DEBUG("ROS_MASTER_URI not set, searching...")
                // primitive search for a rosmaster advertised with zeroconf (Bonjour)
                let browser = RosMasterBrowser()
                browser.start()
                while browser.host.isEmpty {
                    RunLoop.main.run(until: .init(timeIntervalSinceNow: 1.0))
                }
                if browser.host != "error" {
                    masterUriEnv = "http://\(browser.host):\(browser.port)"
                } else if amIBeingDebugged() {
                    masterUriEnv = "http://\(RosNetwork.determineHost()):11311"
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
            masterURI = masterUriEnv
        }
#endif

        // Split URI into
        guard let master = RosNetwork.splitURI(uri: masterURI!) else {
            fatalError( "Couldn't parse the master URI [\(masterURI!)] into a host:port pair.")
        }
        
        return master
    }

    init(group: EventLoopGroup, host: String, port: UInt16) {
        masterHost = host
        masterPort = port

        self.group = group
        self.bootstrap = ClientBootstrap(group: group)
            // Enable SO_REUSEADDR.
            .channelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
            .channelInitializer { channel in
                channel.pipeline.addHandlers([ByteToMessageHandler(XmlRpcMessageDelimiterCodec()),
                                              XmlRpcHandler(owner: self)])
        }
        
        ROS_DEBUG("master on host: \(masterHost), port: \(masterPort)")

    }

    enum ValidateError: Error {
        case malformed(String)
        case error(String)
    }

    func validateXmlrpcResponse(method: String, response: XmlRpcValue) -> Result<XmlRpcValue,ValidateError> {

        guard response.isArray else {
            return .failure(.malformed("XML-RPC call [\(method)] didn't return an array"))
        }

        var resp = response
        if resp.size() == 1 {
            resp = resp[0]
        }

        if resp.size() != 2 && resp.size() != 3 {
            return .failure(.malformed("XML-RPC call [\(method)] didn't return a 2 or 3-element array"))
        }
        guard case .int(let statusCode) = resp[0] else {
            return .failure(.malformed("XML-RPC call [\(method)] didn't return a int as the 1st element"))
        }
        guard case .string(let statusString) = resp[1] else {
            return .failure(.malformed("XML-RPC call [\(method)] didn't return a string as the 2nd element"))
        }
        if statusCode != 1 {
            return .failure(.error("XML-RPC call [\(method)] returned an error (\(statusCode): [\(statusString)]"))
        }
        if resp.size() > 2 {
            if case .string(let uri) = resp[2] {
                return .success(XmlRpcValue(anyArray: [uri]))
            } else {
                return .success(resp[2])
            }
        }
        return .success(XmlRpcValue(anyArray: []))
    }

    func generateHeader(body: String, host: String, port: UInt16) -> String {
        let header = "POST "
            + uri
            + " HTTP/1.1\r\nUser-Agent: "
            + xmlrpcVersion
            + "\r\nHost: "
            + host
            + ":\(port)\r\n"
            + "Content-Type: text/xml\r\nContent-length: "
            + "\(body.lengthOfBytes(using: .utf8))\r\n\r\n"
        return header
    }

    func generateRequest(methodName: String, params: XmlRpcValue, host: String, port: UInt16) -> String {
        var body = XMLRPCClient.requestBegin + methodName + Tags.endMethodname.rawValue

        // If params is an array, each element is a separate parameter
        if params.valid() {
            body += Tags.params.rawValue
            if case .array(let array) = params {
                for a in array {
                    body += Tags.param.rawValue
                    body += a.toXml()
                    body += Tags.endParam.rawValue
                }
            } else {
                body += Tags.param.rawValue
                body += params.toXml()
                body += Tags.endParam.rawValue
            }

            body += Tags.endParams.rawValue
        }
        body += XMLRPCClient.requestEnd

        let header = generateHeader(body: body, host: host, port: port)
        let request = header + body
        return request
    }

    func execute(method: String, request: XmlRpcValue) -> EventLoopFuture<XmlRpcValue> {
        return execute(method: method, request: request, host: masterHost, port: masterPort)
    }

    enum MasterError: Error {
        case invalidResponse(String)
        case writeError(String)
    }

    func execute(method: String, request: XmlRpcValue, host: String, port: UInt16) -> EventLoopFuture<XmlRpcValue> {
        let xml = generateRequest(methodName: method, params: request, host: host, port: port)

        let eventLoop = group.next()
        let promise: EventLoopPromise<XmlRpcValue> = eventLoop.makePromise()

        bootstrap?.connect(host: host, port: Int(port)).map { channel -> Void in
            let buffer = channel.allocator.buffer(string: xml)
            channel.writeAndFlush(buffer).whenFailure { error in
                ROS_ERROR("write failed to \(channel.remoteAddress!) [\(error)]")
                promise.fail(MasterError.writeError("write failed to \(channel.remoteAddress!) [\(error)]"))
            }

            channel.closeFuture.whenComplete { res in
                // FIXME: check result

                let result = self.lock.withLock { () -> Result<XmlRpcValue, ValidateError> in
                    guard let handler = self.handlers[ObjectIdentifier(channel)] else {
                        fatalError("failed to connect to \(host):\(port) for method \(method)")
                    }

                    return self.validateXmlrpcResponse(method: method, response: handler.response)
                }

                self.unregisterHandler(for: channel)

                switch result {
                case .success(let r):
                    promise.succeed(r)
                case .failure(let err):
                    promise.fail(err)
                }
            }
        }.whenFailure { error in
            ROS_ERROR(error.localizedDescription)
            promise.fail(error)
        }
        return promise.futureResult

    }

    func getTopics(callerId: String) -> EventLoopFuture<[TopicInfo]> {
        let args = XmlRpcValue(strings: callerId, "")
        return execute(method: "getPublishedTopics", request: args).map({ (rpc) -> [TopicInfo] in
            return rpc.map { TopicInfo(name: $0[0].string, dataType: $0[1].string )}
        })
    }

    func getNodes(callerId: String) -> EventLoopFuture<[String]> {
        let args = XmlRpcValue(strings: callerId)
        return execute(method: "getSystemState", request: args).map({ (rpc) -> [String] in
            var nodes = Set<String>()
            
            for i in 0..<rpc.size() {
                for j in 0..<rpc[i].size() {
                    let val = rpc[i][j][1]
                    for k in 0..<val.size() {
                        let name = rpc[i][j][1][k]
                        nodes.insert(name.string)
                    }
                }
            }
            
            
            return Array(nodes).sorted()
        })
    }


    

}
