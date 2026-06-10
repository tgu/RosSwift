//
//  RosLogging.swift
//  RosSwift
//
//  swift-log integration helpers, including an Apple unified-logging
//  (`os.Logger`) backend.
//

import Logging

#if canImport(os)
import os

/// A swift-log `LogHandler` that forwards to Apple's unified logging system
/// (`os.Logger`), so RosSwift logs show up in Console.app / `log stream` and
/// participate in the system log archive.
///
/// Available on Apple platforms only. On Linux use `StreamLogHandler`.
public struct OSLogHandler: LogHandler {

    public var logLevel: Logging.Logger.Level = .info
    public var metadata: Logging.Logger.Metadata = [:]

    private let oslogger: os.Logger

    /// - Parameters:
    ///   - label: The logger label; used as the os_log `category`.
    ///   - subsystem: The os_log subsystem (typically a reverse-DNS app id).
    public init(label: String, subsystem: String) {
        self.oslogger = os.Logger(subsystem: subsystem, category: label)
    }

    public subscript(metadataKey key: String) -> Logging.Logger.Metadata.Value? {
        get { metadata[key] }
        set { metadata[key] = newValue }
    }

    public func log(level: Logging.Logger.Level,
                    message: Logging.Logger.Message,
                    metadata: Logging.Logger.Metadata?,
                    source: String,
                    file: String,
                    function: String,
                    line: UInt) {
        let merged = self.metadata.merging(metadata ?? [:]) { $1 }
        let text: String
        if merged.isEmpty {
            text = message.description
        } else {
            let meta = merged.map { "\($0)=\($1)" }.sorted().joined(separator: " ")
            text = "\(message) [\(meta)]"
        }
        oslogger.log(level: level.osLogType, "\(text, privacy: .public)")
    }
}

extension Logging.Logger.Level {
    /// Maps a swift-log level onto the closest `OSLogType`.
    var osLogType: OSLogType {
        switch self {
        case .trace, .debug: return .debug
        case .info, .notice: return .info
        case .warning: return .default
        case .error: return .error
        case .critical: return .fault
        }
    }
}
#endif

/// Convenience swift-log bootstrap for RosSwift applications.
public enum RosLogging {

    /// Bootstraps swift-log with a platform-appropriate handler: Apple unified
    /// logging (`os.Logger`) on Apple platforms, `StreamLogHandler` (stdout)
    /// elsewhere. Call once, early in process start.
    ///
    /// The handler passes all levels through, so each `Logger`'s own
    /// `logLevel` governs filtering (and RosSwift's `~set_logger_level`
    /// service takes effect).
    ///
    /// - Parameter subsystem: The os_log subsystem on Apple platforms.
    public static func bootstrap(subsystem: String = "org.ros.rosswift") {
#if canImport(os)
        LoggingSystem.bootstrap { label in
            var handler = OSLogHandler(label: label, subsystem: subsystem)
            handler.logLevel = .trace
            return handler
        }
#else
        LoggingSystem.bootstrap { label in
            var handler = StreamLogHandler.standardOutput(label: label)
            handler.logLevel = .trace
            return handler
        }
#endif
    }
}
