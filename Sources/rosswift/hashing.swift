//
//  hashing.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-12.
//

#if os(OSX)
import Foundation
import CommonCrypto

extension Data {

    /// Hashing algorithm that prepends an RSA2048ASN1Header to the beginning of the data being hashed.
    ///
    /// - Parameters:
    ///   - type: The type of hash algorithm to use for the hashing operation.
    ///   - output: The type of output string desired.
    /// - Returns: A hash string using the specified hashing algorithm, or nil.
    public func hashWithRSA2048Asn1Header(_ type: HashType, output: HashOutputType = .hex) -> String? {

        let rsa2048Asn1Header:[UInt8] = [
            0x30, 0x82, 0x01, 0x22, 0x30, 0x0d, 0x06, 0x09, 0x2a, 0x86, 0x48, 0x86,
            0xf7, 0x0d, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0f, 0x00
        ]

        var headerData = Data(bytes: rsa2048Asn1Header)
        headerData.append(self)

        return hashed(type, output: output)
    }

    /// Hashing algorithm for hashing a Data instance.
    ///
    /// - Parameters:
    ///   - type: The type of hash to use.
    ///   - output: The type of hash output desired, defaults to .hex.
    ///   - Returns: The requested hash output or nil if failure.
    public func hashed(_ type: HashType, output: HashOutputType = .hex) -> String? {

        // setup data variable to hold hashed value
        var digest = Data(count: Int(type.length))

        // generate hash using specified hash type
        _ = digest.withUnsafeMutableBytes { (digestBytes: UnsafeMutablePointer<UInt8>) in
            self.withUnsafeBytes { (messageBytes: UnsafePointer<UInt8>) in
                let length = CC_LONG(self.count)
                switch type {
                case .md5: CC_MD5(messageBytes, length, digestBytes)
                case .sha1: CC_SHA1(messageBytes, length, digestBytes)
                case .sha224: CC_SHA224(messageBytes, length, digestBytes)
                case .sha256: CC_SHA256(messageBytes, length, digestBytes)
                case .sha384: CC_SHA384(messageBytes, length, digestBytes)
                case .sha512: CC_SHA512(messageBytes, length, digestBytes)
                }
            }
        }

        // return the value based on the specified output type.
        switch output {
        case .hex: return digest.map { String(format: "%02hhx", $0) }.joined()
        case .base64: return digest.base64EncodedString()
        }
    }
}

// Defines types of hash string outputs available
public enum HashOutputType {
    // standard hex string output
    case hex
    // base 64 encoded string output
    case base64
}

// Defines types of hash algorithms available
public enum HashType {
    case md5
    case sha1
    case sha224
    case sha256
    case sha384
    case sha512

    var length: Int32 {
        switch self {
        case .md5: return CC_MD5_DIGEST_LENGTH
        case .sha1: return CC_SHA1_DIGEST_LENGTH
        case .sha224: return CC_SHA224_DIGEST_LENGTH
        case .sha256: return CC_SHA256_DIGEST_LENGTH
        case .sha384: return CC_SHA384_DIGEST_LENGTH
        case .sha512: return CC_SHA512_DIGEST_LENGTH
        }
    }
}

public extension String {

    /// Hashing algorithm for hashing a string instance.
    ///
    /// - Parameters:
    ///   - type: The type of hash to use.
    ///   - output: The type of output desired, defaults to .hex.
    /// - Returns: The requested hash output or nil if failure.
    public func hashed(_ type: HashType, output: HashOutputType = .hex) -> String? {

        // convert string to utf8 encoded data
        guard let message = data(using: .utf8) else { return nil }
        return message.hashed(type, output: output)
    }
}
#endif
