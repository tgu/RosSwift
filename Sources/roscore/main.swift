import Foundation
import Logging
import rosmaster
import RosNetwork

public class MyLog: LogHandler {
    let label: String

    public func log(level: Logger.Level, message: Logger.Message, metadata: Logger.Metadata?, file: String, function: String, line: UInt) {
        print("\(label): \(message)")
    }

    public subscript(metadataKey _: String) -> Logger.Metadata.Value? {
        get { return nil }
        set(newValue) { }
    }

    public init(label: String) {
        metadata = [:]
        logLevel = .debug
        self.label = label
    }

    public var metadata: Logger.Metadata
    public var logLevel: Logger.Level
}

LoggingSystem.bootstrap(MyLog.init)

fileprivate var logger = Logger(label: "roscore")


private func trap(signal sig: Signal, handler: @escaping (Signal) -> Void) -> DispatchSourceSignal {
    let queue = DispatchQueue(label: "rosmaster")
    let signalSource = DispatchSource.makeSignalSource(signal: sig.rawValue, queue: queue)
    signal(sig.rawValue, SIG_IGN)
    signalSource.setEventHandler(handler: {
        signalSource.cancel()
        handler(sig)
    })
    signalSource.resume()
    return signalSource
}

private enum Signal: Int32 {
    case HUP = 1
    case INT = 2
    case QUIT = 3
    case ABRT = 6
    case KILL = 9
    case ALRM = 14
    case TERM = 15
}

let network = RosNetwork(remappings: [:])
let master = Master(host: network.gHost, port: defaultMasterPort)

_ = try! master.start().wait()

let message = "started roslaunch server http://\(network.gHost)/".bold()
logger.info("\(message)")
logger.info("rosmaster version for Swift 5")

private let group = DispatchGroup()
group.enter()
let signalSource = trap(signal: Signal.INT) { signal in
    logger.debug("intercepted signal: \(signal)")
    master.stop().whenComplete { _ in
        group.leave()
    }
}

let terminationSource = trap(signal: Signal.TERM) { signal in
    logger.debug("intercepted signal: \(signal)")
    master.stop().whenComplete { _ in
        group.leave()
    }
}

group.wait()
// cleanup
signalSource.cancel()
terminationSource.cancel()

