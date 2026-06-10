//
//  inboundConnection.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-23.
//

import BinaryCoder
import NIO
import NIOCore
import Atomics
import NIOExtras
import StdMsgs
import Synchronization

enum ConnectionError: Error {
    case connectionDropped
}

final class InboundConnection: Sendable {

    unowned let parent: Subscription
    let host: String
    let port: Int

    private let dropped = ManagedAtomic<Bool>(false)
    private let localPort = ManagedAtomic<Int32>(0)
    private let runTask = Mutex<Task<Void, Never>?>(nil)

    init(parent: Subscription, host: String, port: Int) {
        self.host = host
        self.port = port
        self.parent = parent
    }

    deinit {
        ROS_DEBUG("InboundConnection deinit")
    }

    /// Connects to the publisher, performs the TCPROS handshake by writing
    /// `headerKeyVals` and reading the publisher's response header, then loops
    /// delivering received message frames to `parent.handle(...)`.
    ///
    /// Returns once the connection is established and the message-loop Task has
    /// been spawned. Cancellation of that Task happens via `dropConnection`.
    func initialize(owner: TransportPublisherLink, headerKeyVals: StringStringMap) async {
        do {
            let asyncChannel = try await connect()
            if let lp = asyncChannel.channel.localAddress?.port {
                localPort.store(Int32(lp), ordering: .relaxed)
            }
            // If dropConnection has already been called, don't start the loop.
            if dropped.load(ordering: .relaxed) {
                asyncChannel.channel.close(promise: nil)
                return
            }
            let task = Task { [self] in
                await run(asyncChannel: asyncChannel, link: owner, headerKeyVals: headerKeyVals)
            }
            runTask.withLock { $0 = task }
            // Re-check drop in case it happened concurrently with the Task spawn.
            if dropped.load(ordering: .relaxed) {
                runTask.withLock { $0?.cancel(); $0 = nil }
            }
        } catch {
            ROS_ERROR("InboundConnection bootstrap to [\(host):\(port)] failed: \(error)")
        }
    }

    private func connect() async throws -> NIOAsyncChannel<ByteBuffer, ByteBuffer> {
        try await ClientBootstrap(group: threadGroup)
            .channelOption(ChannelOptions.socket(SocketOptionLevel(SOL_SOCKET), SO_REUSEADDR), value: 1)
            .connect(host: host, port: port) { channel in
                channel.eventLoop.makeCompletedFuture {
                    try channel.pipeline.syncOperations.addHandler(
                        ByteToMessageHandler(LengthFieldBasedFrameDecoder(lengthFieldLength: .four,
                                                                          lengthFieldEndianness: .little))
                    )
                    return try NIOAsyncChannel<ByteBuffer, ByteBuffer>(
                        wrappingChannelSynchronously: channel,
                        configuration: .init()
                    )
                }
            }
    }

    private func run(
        asyncChannel: NIOAsyncChannel<ByteBuffer, ByteBuffer>,
        link: TransportPublisherLink,
        headerKeyVals: StringStringMap
    ) async {
        do {
            try await asyncChannel.executeThenClose { inbound, outbound in
                // 1. Send TCPROS subscriber header.
                let headerBytes = Header.write(keyVals: headerKeyVals)
                var headerFrame = ByteBufferAllocator().buffer(capacity: 4 + headerBytes.count)
                headerFrame.writeInteger(UInt32(headerBytes.count), endianness: .little)
                headerFrame.writeBytes(headerBytes)
                try await outbound.write(headerFrame)
                ROS_DEBUG("Header written for topic [\(link.parent.name)]")

                // 2. Read publisher's response header (first frame).
                var iterator = inbound.makeAsyncIterator()
                guard var firstFrame = try await iterator.next() else {
                    ROS_DEBUG("InboundConnection: peer closed before header from \(self.remoteAddressString)")
                    return
                }
                guard let respBytes = firstFrame.readBytes(length: firstFrame.readableBytes),
                      let header = Header(buffer: respBytes) else {
                    ROS_ERROR("InboundConnection: could not parse publisher header from \(self.remoteAddressString)")
                    return
                }
                if let errMsg = header["error"] {
                    ROS_ERROR("Received error in header for connection to [\(self.remoteAddressString)]: [\(errMsg)]")
                    return
                }
                if !link.setHeader(header: header) {
                    ROS_ERROR("InboundConnection: setHeader rejected publisher header")
                    return
                }
                guard let connectionHeader = link.header?.headers else {
                    return
                }

                // 3. Message loop.
                while var frame = try await iterator.next() {
                    guard let bytes = frame.readBytes(length: frame.readableBytes) else { continue }
                    let msg = SerializedMessage(buffer: bytes)
                    _ = await parent.handle(message: msg,
                                            connectionHeader: connectionHeader,
                                            link: link)
                }
            }
        } catch is CancellationError {
            ROS_DEBUG("InboundConnection to \(remoteAddressString) cancelled")
        } catch {
            ROS_DEBUG("InboundConnection to \(remoteAddressString) error: \(error)")
        }
    }

    func dropConnection(reason: DropReason) {
        if dropped.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged {
            ROS_DEBUG("Connection::drop - \(reason)")
            runTask.withLock {
                $0?.cancel()
                $0 = nil
            }
        }
    }

    var remoteAddressString: String {
        "\(host):\(port)"
    }

    func getTransportInfo() -> String {
        "TCPROS connection on port \(localPort.load(ordering: .relaxed)) to [\(remoteAddressString)]"
    }
}
