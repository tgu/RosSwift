//
//  DecoderTest.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2019-04-03.
//

import Testing
import NIO
@testable import RosSwift
@testable import RosTime
import BinaryCoder

private let standardDataString = "abcde"

@Suite("Decoder tests")
struct DecoderTest {

    @Test func headerWithData() throws {
        let channel = EmbeddedChannel()
        let decoderUnderTest = ByteToMessageHandler(MessageDelimiterCodec())
        try channel.pipeline.syncOperations.addHandler(decoderUnderTest)

        let dataLength: UInt32 = 5

        var buffer = channel.allocator.buffer(capacity: 9) // 4 byte header + 5 character string
        buffer.writeInteger(dataLength, endianness: .little, as: UInt32.self)
        buffer.writeString(standardDataString)

        #expect(try channel.writeInbound(buffer).isFull)

        let data = try (channel.readInbound(as: ByteBuffer.self)?.readableBytesView).map {
            String(decoding: $0, as: Unicode.UTF8.self)
        }
        #expect(data != nil)

        #expect(standardDataString == String(data!.dropFirst(4)))
        #expect(try channel.finish().isClean)
    }

    @Test func decodeTwoFrames() throws {
        let channel = EmbeddedChannel()
        let decoderUnderTest = ByteToMessageHandler(MessageDelimiterCodec())
        try channel.pipeline.syncOperations.addHandler(decoderUnderTest)

        let firstFrameDataLength: UInt32 = 5
        let secondFrameDataLength: UInt32 = 3
        let secondFrameString = "123"

        var buffer = channel.allocator.buffer(capacity: 16) // 4 byte header + 5 character string + 4 byte header + 3 character string
        buffer.writeInteger(firstFrameDataLength, endianness: .little, as: UInt32.self)
        buffer.writeString(standardDataString)
        buffer.writeInteger(secondFrameDataLength, endianness: .little, as: UInt32.self)
        buffer.writeString(secondFrameString)

        #expect(try channel.writeInbound(buffer).isFull)

        let firstFrame = try channel.readInbound(as: ByteBuffer.self)?.readableBytesView.dropFirst(4)
        #expect(firstFrame.map { String(decoding: $0, as: Unicode.UTF8.self) } == standardDataString)

        let secondFrame = try channel.readInbound(as: ByteBuffer.self)?.readableBytesView.dropFirst(4)
        #expect(secondFrame.map { String(decoding: $0, as: Unicode.UTF8.self) } == secondFrameString)

        #expect(try channel.finish().isClean)
    }

    @Test func decodeSplitIncomingData() throws {
        let channel = EmbeddedChannel()
        let decoderUnderTest = ByteToMessageHandler(MessageDelimiterCodec())
        try channel.pipeline.syncOperations.addHandler(decoderUnderTest)

        let frameDataLength: UInt32 = 5

        // Write and try to read both bytes of the data individually
        let frameDataLengthFirstByte: UInt8 = UInt8(frameDataLength)
        let frameDataLengthSecondByte: UInt8 = 0

        var firstBuffer = channel.allocator.buffer(capacity: 1) // Byte 1 of 2 byte header header
        firstBuffer.writeInteger(frameDataLengthFirstByte, endianness: .little, as: UInt8.self)

        #expect(try channel.writeInbound(firstBuffer).isEmpty)

        // Read should fail because there is not yet enough data.
        #expect(try channel.readInbound(as: ByteBuffer.self) == nil)

        var secondBuffer = channel.allocator.buffer(capacity: 1) // Byte 2 of 2 byte header header
        secondBuffer.writeInteger(frameDataLengthSecondByte, endianness: .little, as: UInt8.self)
        secondBuffer.writeInteger(frameDataLengthSecondByte, endianness: .little, as: UInt8.self)
        secondBuffer.writeInteger(frameDataLengthSecondByte, endianness: .little, as: UInt8.self)

        #expect(try channel.writeInbound(secondBuffer).isEmpty)

        // Read should fail because there is not yet enough data.
        #expect(try channel.readInbound(as: ByteBuffer.self) == nil)

        // Write and try to read each byte of the data individually
        for (index, character) in standardDataString.enumerated() {

            var characterBuffer = channel.allocator.buffer(capacity: 1)
            characterBuffer.writeString(String(character))

            if index < standardDataString.count - 1 {

                #expect(try channel.writeInbound(characterBuffer).isEmpty)
                // Read should fail because there is not yet enough data.
                #expect(try channel.readInbound(as: ByteBuffer.self) == nil)
            } else {
                #expect(try channel.writeInbound(characterBuffer).isFull)
            }
        }

        let frame = try channel.readInbound(as: ByteBuffer.self)?.readableBytesView.dropFirst(4)
        #expect(frame.map { String(decoding: $0, as: Unicode.UTF8.self) } == standardDataString)
        #expect(try channel.finish().isClean)
    }

    @Test func emptyBuffer() throws {
        let channel = EmbeddedChannel()
        let decoderUnderTest = ByteToMessageHandler(MessageDelimiterCodec())
        try channel.pipeline.syncOperations.addHandler(decoderUnderTest)

        let buffer = channel.allocator.buffer(capacity: 1)
        #expect(try channel.writeInbound(buffer).isEmpty)
        #expect(try channel.finish().isClean)
    }

    @Test func time() throws {
        let time = Time(sec: 1, nsec: 2)
        let data = try BinaryEncoder.encode(time)
        let t2 = try BinaryDecoder.decode(Time.self, data: data)
        #expect(time == t2)
    }

    @Test func time2() throws {
        let time = Time(sec: 91223, nsec: UInt32.max)
        let data = try BinaryEncoder.encode(time)
        let t2 = try BinaryDecoder.decode(Time.self, data: data)
        #expect(time == t2)
    }
}
