import Foundation

#if os(macOS) || os(iOS) || os(tvOS) || os(watchOS)
import CommonCrypto
#elseif os(Linux)
import OpenSSL
typealias CC_LONG = size_t
#endif


extension Data {
    /// MD5 Hashing algorithm for hashing a Data instance.
    ///
    /// - Returns: The requested hash output or nil if failure.
    public func hashed() -> String {

        #if os(macOS) || os(iOS) || os(tvOS) || os(watchOS)
        let length = CC_MD5_DIGEST_LENGTH
        #elseif os(Linux)
        let length = MD5_DIGEST_LENGTH
        #endif

        var digest = Data(count: Int(length))
        
        digest.withUnsafeMutableBytes { (digestBytes) in
            self.withUnsafeBytes { (messageBytes) in
                let length = CC_LONG(self.count)
                #if os(macOS) || os(iOS) || os(tvOS) || os(watchOS)
                CC_MD5(messageBytes.baseAddress!, length, digestBytes.bindMemory(to: UInt8.self).baseAddress!)
                #elseif os(Linux)
                MD5(messageBytes.baseAddress?.assumingMemoryBound(to: UInt8.self), length, digestBytes.bindMemory(to: UInt8.self).baseAddress!)
                #endif
            }
        }

        // return the value based on the specified output type.
        return digest.map { String(format: "%02hhx", $0) }.joined()
    }
}


public extension String {

    /// MD5 Hashing algorithm for hashing a string instance.
    /// - Returns: The requested hash output or nil if failure.
    func hashed() -> String? {

        // convert string to utf8 encoded data
        return data(using: .utf8)?.hashed()
    }
}

