//
//  Master.swift
//  swiftros
//
//  Created by Thomas Gustafsson on 2018-03-19.
//

import Foundation
import NIO
import NIOCore
import NIOConcurrencyHelpers
import rpcobject
import RosNetwork
import Synchronization
#if os(Linux)
//import NetService
#endif

let threadGroup = MultiThreadedEventLoopGroup(numberOfThreads: 1)

/// Topic information
///
/// The type is represented in the format "package_name/MessageName"

public struct TopicInfo: Sendable {
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

        guard content.firstIndex(of: "<") != nil else {
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



final class Master: Sendable {

    let masterHost: String
    let masterPort: UInt16
    let group: EventLoopGroup

    var path: String {
        return "\(masterHost):\(masterPort)"
    }

    var uri: String {
        return "/"
    }

    static func determineRosMasterAddress(remappings: StringStringMap) -> (host: String, port: UInt16) {
        var masterURI = remappings["__master"]
#if os(macOS) || os(iOS) || os(tvOS)
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

    func execute(method: String, request: XmlRpcValue) async throws -> XmlRpcValue {
        return try await execute(method: method, request: request, host: masterHost, port: masterPort)
    }

    enum MasterError: Error {
        case invalidResponse(String)
        case writeError(String)
    }

    func execute(method: String, request: XmlRpcValue, host: String, port: UInt16) async throws -> XmlRpcValue {
        let xml = generateRequest(methodName: method, params: request, host: host, port: port)
        return try await executeAsync(xml: xml, method: method, host: host, port: port)
    }

    /// One-shot XML-RPC request: connect, write the encoded request, read
    /// the single response frame from the codec, parse and validate it.
    private func executeAsync(xml: String, method: String, host: String, port: UInt16) async throws -> XmlRpcValue {
        let asyncChannel: NIOAsyncChannel<ByteBuffer, ByteBuffer> = try await ClientBootstrap(group: group)
            .channelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
            .connect(host: host, port: Int(port)) { channel in
                channel.eventLoop.makeCompletedFuture {
                    try channel.pipeline.syncOperations.addHandler(
                        ByteToMessageHandler(XmlRpcMessageDelimiterCodec())
                    )
                    return try NIOAsyncChannel<ByteBuffer, ByteBuffer>(
                        wrappingChannelSynchronously: channel,
                        configuration: .init()
                    )
                }
            }

        return try await asyncChannel.executeThenClose { inbound, outbound in
            // Write the request.
            let requestBuffer = ByteBuffer(string: xml)
            try await outbound.write(requestBuffer)

            // Read the single framed response from the codec.
            var iterator = inbound.makeAsyncIterator()
            guard var responseFrame = try await iterator.next() else {
                throw MasterError.invalidResponse("peer closed before sending a response (\(method) → \(host):\(port))")
            }
            guard let body = responseFrame.readString(length: responseFrame.readableBytes) else {
                throw MasterError.invalidResponse("non-UTF8 response body from \(host):\(port)")
            }
            guard let range = body.lowercased().range(of: "content-length: ") else {
                throw MasterError.invalidResponse("response missing content-length header")
            }
            let afterHeader = String(body[range.upperBound..<body.endIndex])
            guard let xmlStart = afterHeader.firstIndex(of: "<") else {
                throw MasterError.invalidResponse("response missing XML body")
            }
            let xmlBody = String(afterHeader[xmlStart..<afterHeader.endIndex])

            guard let parsed = XMLRPCClient.parseResponse(xml: xmlBody) else {
                throw MasterError.invalidResponse("could not parse XML-RPC response")
            }

            switch self.validateXmlrpcResponse(method: method, response: parsed) {
            case .success(let r):
                return r
            case .failure(let err):
                throw err
            }
        }
    }

    func getTopics(callerId: String) async throws -> [TopicInfo] {
        let args = XmlRpcValue(strings: callerId, "")
        let rpc = try await execute(method: "getPublishedTopics", request: args)
        return rpc.map { TopicInfo(name: $0[0].string, dataType: $0[1].string )}
    }

    func getNodes(callerId: String) async throws -> [String] {
        let args = XmlRpcValue(strings: callerId)
        let rpc = try await execute(method: "getSystemState", request: args)
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
    }
}
