//
//  Master.swift
//  swiftros
//
//  Created by Thomas Gustafsson on 2018-03-19.
//

import Foundation
import NIO
import Logging
import rpcobject

fileprivate let logger = Logger(label: "master")

let xmlrpcVersion = "XMLRPC++ 0.7"

public enum nio {

    struct TopicInfo {
        let name: String
        let dataType: String
    }

    internal struct RPCResponse {
        let result: XmlRpcValue

        static let methodresponseTag = "<methodResponse>"

        init?(xml: String) {
            guard let tagIndex = xml.range(of: RPCResponse.methodresponseTag) else {
                logger.error("Invalid response - no methodResponse. Response:\n\(xml)")
                return nil
            }

            var result = XmlRpcValue()

            var data = xml.suffix(from: tagIndex.upperBound)
            if XmlRpcUtil.nextTagIs(tag: .params, xml: &data) && XmlRpcUtil.nextTagIs(tag: .param, xml: &data) {
                if result.fromXML(xml: &data) {
                    self.result = result
                    return
                } else {
                    logger.error("Invalid response value. Response: \(data)")
                }
            } else if XmlRpcUtil.nextTagIs(tag: .fault, xml: &data) {
                logger.error("Invalid response value. Response: \(data)")
            } else {
                logger.error("Invalid response - no param or fault tag. Response \(xml)")
            }

            return nil
        }
    }

    internal enum CodecError: Error {
        case headerNotRead
        case malformedHeader
        case lengthError
        case parsingError
    }

    final class XmlRpcMessageDelimiterCodec: ByteToMessageDecoder {
        public typealias InboundIn = ByteBuffer
        public typealias InboundOut = ByteBuffer

        public var cumulationBuffer: ByteBuffer?

        public func decode(context: ChannelHandlerContext, buffer: inout ByteBuffer) throws -> DecodingState {

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
                logger.debug("Malformed header")
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

    struct XMLRPCClient {
        static let requestBegin = "<?xml version=\"1.0\"?>\r\n<methodCall>\r\n<methodName>"
        static let requestEnd = "</methodCall>\r\n"

        static func generateHeader(body: String, host: String, port: UInt16) -> String {
            let header = "POST "
                + "/"
                + " HTTP/1.1\r\nUser-Agent: "
                + xmlrpcVersion
                + "\r\nHost: "
                + host
                + ":\(port)\r\n"
                + "Content-Type: text/xml\r\nContent-length: "
                + "\(body.lengthOfBytes(using: .utf8))\r\n\r\n"
            return header
        }

        static func generateRequest(methodName: String, params: XmlRpcValue, host: String, port: UInt16) -> String {
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
    }


    public final class Master: Sendable {
        let group: EventLoopGroup

        public init(group: EventLoopGroup) {
            self.group = group
        }

        /// One-shot XML-RPC request to `host:port`: open a NIOAsyncChannel,
        /// write the encoded request, read the single framed response, parse
        /// and return it. The connection is scoped to this call.
        public func send(method: String, request: XmlRpcValue, host: String, port: Int) async throws -> XmlRpcValue {
            let asyncChannel: NIOAsyncChannel<ByteBuffer, ByteBuffer> = try await ClientBootstrap(group: self.group)
                .channelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
                .connect(host: host, port: port) { channel in
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
                let xml = XMLRPCClient.generateRequest(
                    methodName: method,
                    params: request,
                    host: host,
                    port: UInt16(port)
                )
                try await outbound.write(ByteBuffer(string: xml))

                var iterator = inbound.makeAsyncIterator()
                guard var frame = try await iterator.next() else {
                    throw ClientError.connectionResetByPeer
                }
                guard let body = frame.readString(length: frame.readableBytes) else {
                    throw CodecError.parsingError
                }
                guard let response = RPCResponse(xml: body) else {
                    throw CodecError.parsingError
                }
                return response.result
            }
        }

        public enum ClientError: Error {
            case notReady
            case cantBind
            case timeout
            case connectionResetByPeer
        }
    }

}
