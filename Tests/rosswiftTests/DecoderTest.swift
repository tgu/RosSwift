//
//  DecoderTest.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2019-04-03.
//

import XCTest
import NIO
@testable import RosSwift

private let standardDataString = "abcde"

class DecoderTest: XCTestCase {
    private var channel: EmbeddedChannel!
    private var decoderUnderTest: ByteToMessageHandler<MessageDelimiterCodec>!


    override func setUp() {
        self.channel = EmbeddedChannel()
    }

    override func tearDown() {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }

    func testHeaderWithData() {
        self.decoderUnderTest = .init(MessageDelimiterCodec())
        XCTAssertNoThrow(try self.channel.pipeline.addHandler(self.decoderUnderTest).wait())

        let dataLength: UInt32 = 5

        var buffer = self.channel.allocator.buffer(capacity: 9) // 4 byte header + 5 character string
        buffer.writeInteger(dataLength, endianness: .little, as: UInt32.self)
        buffer.writeString(standardDataString)

        XCTAssertTrue(try self.channel.writeInbound(buffer).isFull)

        let data = try! (self.channel.readInbound(as: ByteBuffer.self)?.readableBytesView).map {
            String(decoding: $0, as: Unicode.UTF8.self)
        }
        XCTAssertNotNil(data)

        XCTAssertEqual(standardDataString,String(data!.dropFirst(4)))
        XCTAssertTrue(try self.channel.finish().isClean)
    }

    func testDecodeTwoFrames() throws {

        self.decoderUnderTest = .init(MessageDelimiterCodec())
        XCTAssertNoThrow(try self.channel.pipeline.addHandler(self.decoderUnderTest).wait())

        let firstFrameDataLength: UInt32 = 5
        let secondFrameDataLength: UInt32 = 3
        let secondFrameString = "123"

        var buffer = self.channel.allocator.buffer(capacity: 16) // 4 byte header + 5 character string + 4 byte header + 3 character string
        buffer.writeInteger(firstFrameDataLength, endianness: .little, as: UInt32.self)
        buffer.writeString(standardDataString)
        buffer.writeInteger(secondFrameDataLength, endianness: .little, as: UInt32.self)
        buffer.writeString(secondFrameString)

        XCTAssertTrue(try self.channel.writeInbound(buffer).isFull)
        XCTAssertNoThrow(XCTAssertEqual(standardDataString,
                                        try (self.channel.readInbound(as: ByteBuffer.self)?.readableBytesView.dropFirst(4)).map {
                                            String(decoding: $0, as: Unicode.UTF8.self)
            }))

        XCTAssertNoThrow(XCTAssertEqual(secondFrameString,
                                        try (self.channel.readInbound(as: ByteBuffer.self)?.readableBytesView.dropFirst(4)).map {
                                            String(decoding: $0, as: Unicode.UTF8.self)
            }))

        XCTAssertTrue(try self.channel.finish().isClean)
    }

    func testDecodeSplitIncomingData() throws {

        self.decoderUnderTest = .init(MessageDelimiterCodec())
        XCTAssertNoThrow(try self.channel.pipeline.addHandler(self.decoderUnderTest).wait())

        let frameDataLength: UInt32 = 5

        // Write and try to read both bytes of the data individually
        let frameDataLengthFirstByte: UInt8 = UInt8(frameDataLength)
        let frameDataLengthSecondByte: UInt8 = 0

        var firstBuffer = self.channel.allocator.buffer(capacity: 1) // Byte 1 of 2 byte header header
        firstBuffer.writeInteger(frameDataLengthFirstByte, endianness: .little, as: UInt8.self)

        XCTAssertTrue(try self.channel.writeInbound(firstBuffer).isEmpty)

        // Read should fail because there is not yet enough data.
        XCTAssertNoThrow(XCTAssertNil(try self.channel.readInbound()))

        var secondBuffer = self.channel.allocator.buffer(capacity: 1) // Byte 2 of 2 byte header header
        secondBuffer.writeInteger(frameDataLengthSecondByte, endianness: .little, as: UInt8.self)
        secondBuffer.writeInteger(frameDataLengthSecondByte, endianness: .little, as: UInt8.self)
        secondBuffer.writeInteger(frameDataLengthSecondByte, endianness: .little, as: UInt8.self)

        XCTAssertTrue(try self.channel.writeInbound(secondBuffer).isEmpty)

        // Read should fail because there is not yet enough data.
        XCTAssertNoThrow(XCTAssertNil(try self.channel.readInbound()))

        // Write and try to read each byte of the data individually
        for (index, character) in standardDataString.enumerated() {

            var characterBuffer = self.channel.allocator.buffer(capacity: 1)
            characterBuffer.writeString(String(character))

            if index < standardDataString.count - 1 {

                XCTAssertTrue(try self.channel.writeInbound(characterBuffer).isEmpty)
                // Read should fail because there is not yet enough data.
                XCTAssertNoThrow(XCTAssertNil(try self.channel.readInbound()))
            } else {
                XCTAssertTrue(try self.channel.writeInbound(characterBuffer).isFull)
            }
        }

        XCTAssertNoThrow(XCTAssertEqual(standardDataString,
                                        try (self.channel.readInbound(as: ByteBuffer.self)?.readableBytesView.dropFirst(4)).map {
                                            String(decoding: $0, as: Unicode.UTF8.self)
            }))
        XCTAssertTrue(try self.channel.finish().isClean)
    }

    func testEmptyBuffer() throws {

        self.decoderUnderTest = .init(MessageDelimiterCodec())
        XCTAssertNoThrow(try self.channel.pipeline.addHandler(self.decoderUnderTest).wait())

        let buffer = self.channel.allocator.buffer(capacity: 1)
        XCTAssertTrue(try self.channel.writeInbound(buffer).isEmpty)
        XCTAssertTrue(try self.channel.finish().isClean)
    }



}
