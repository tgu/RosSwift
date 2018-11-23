//
//  Spinner.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-11-22.
//

import Foundation
import NIO
import NIOConcurrencyHelpers
import RosTime

/// Protocol for classes which spin on a callback queue.
protocol Spinner {

    /// Spin on a callback queue (defaults to the global one). Blocks until rosswift
    /// has been shutdown.

    func spin(queue: CallbackQueue?)
}

/// Spinner which runs in a single thread.

class SingleThreadSpinner: Spinner {

    func spin(queue: CallbackQueue? = nil) {
        let useQueue = queue != nil ? queue! : Ros.getGlobalCallbackQueue()
        if !SpinnerMonitor.main.add(queue: useQueue, singleThreaded: true) {
            fatalError("SingleThreadedSpinner: \(DEFAULT_ERROR_MESSAGE) You might want to use a MultiThreadedSpinner instead.")
        }

        let timeout = RosTime.WallDuration(milliseconds: 100)
        let n = Ros.NodeHandle()
        while n.isOK {
            useQueue.callAvailable(timeout: timeout)
        }
        SpinnerMonitor.main.remove(queue: useQueue)
    }
}

/// Spinner which spins in multiple threads.

class MultiThreadedSpinner: Spinner {
    private let threadCount: Int

    ///
    /// - Parameter threadCount: Number of threads to use for calling callbacks.
    /// 0 will automatically use however many hardware threads exist on your system.

    init(threadCount: Int = 0) {
        self.threadCount = threadCount
    }

    func spin(queue: CallbackQueue? = nil) {

    }
}

class AsyncSpinnner {
    private let mutex = DispatchQueue(label: "mutex")
    private var threads = [Thread]()
    private let threadCount: Int
    private let callbackQueue: CallbackQueue
    private var running = Atomic<Bool>(value: false)
    private let node = Ros.NodeHandle()

    init(threadCount: Int, queue: CallbackQueue? = nil) {
        self.callbackQueue = queue != nil ? queue! : Ros.getGlobalCallbackQueue()
        self.threadCount = threadCount != 0 ? threadCount : System.coreCount
    }

    deinit {
        stop()
    }

    func start() {
        guard running.compareAndExchange(expected: false, desired: true) else {
            return
        }

        mutex.sync {
            if !SpinnerMonitor.main.add(queue: callbackQueue, singleThreaded: false) {
                fatalError(DEFAULT_ERROR_MESSAGE)
            }
            for _ in 0..<threadCount {
                let thread = Thread(block: {
                    self.threadFunc()
                })
                threads.append(thread)
                thread.start()
            }
        }
    }

    func stop() {
        guard running.compareAndExchange(expected: true, desired: false) else {
            return
        }

        threads.removeAll()

        SpinnerMonitor.main.remove(queue: callbackQueue)

    }

    private func threadFunc() {
        disableAllSignalsInThisThread()
        let queue = callbackQueue
        let useCallAvailable = threadCount == 1
        let timeout = RosTime.WallDuration(milliseconds: 100)
        while running.load() && node.isOK {
            if useCallAvailable {
                queue.callAvailable(timeout: timeout)
            } else {
                _ = queue.callOne(timeout: timeout)
            }
        }

    }

}


func disableAllSignalsInThisThread() {
    var signal_set = sigset_t()

    /* block all signals */
    sigfillset( &signal_set )
    pthread_sigmask( SIG_BLOCK, &signal_set, nil )
}
