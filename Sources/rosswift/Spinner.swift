//
//  Spinner.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-11-22.
//

import Foundation
import NIO
import Atomics
import RosTime

/// Protocol for classes which spin on a callback queue.
protocol Spinner {

    /// Spin on a callback queue (defaults to the global one). Blocks until rosswift
    /// has been shutdown.

    func spin(ros: Ros, queue: CallbackQueue?)
}

/// Spinner which runs in a single thread.

struct SingleThreadSpinner: Spinner {

    func spin(ros: Ros, queue: CallbackQueue? = nil) {
        let useQueue = queue != nil ? queue! : ros.getGlobalCallbackQueue()
        if !SpinnerMonitor.main.add(queue: useQueue, singleThreaded: true) {
            fatalError("SingleThreadedSpinner: \(DEFAULT_ERROR_MESSAGE) You might want to use a MultiThreadedSpinner instead.")
        }

        let timeout = WallDuration(milliseconds: 100)
        while ros.isRunning.load(ordering: .relaxed) {
            useQueue.callAvailable(timeout: timeout)
        }
        SpinnerMonitor.main.remove(queue: useQueue)
    }
}

final class AsyncSpinnner {
    private let mutex = DispatchQueue(label: "mutex")
    private var threads = [Thread]()
    private let threadCount: Int
    private let callbackQueue: CallbackQueue
    private let running = ManagedAtomic(false)
    private let ros: Ros

    init(ros: Ros, threadCount: Int, queue: CallbackQueue? = nil) {
        self.callbackQueue = queue != nil ? queue! : ros.getGlobalCallbackQueue()
        self.threadCount = threadCount != 0 ? threadCount : System.coreCount
        self.ros = ros
    }

    deinit {
        stop()
    }

    func start() {
        guard running.compareExchange(expected: false, desired: true, ordering: .relaxed).exchanged else {
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
        guard running.compareExchange(expected: true, desired: false, ordering: .relaxed).exchanged else {
            return
        }

        threads.removeAll()

        SpinnerMonitor.main.remove(queue: callbackQueue)

    }

    private func threadFunc() {
        disableAllSignalsInThisThread()
        let queue = callbackQueue
        let useCallAvailable = threadCount == 1
        let timeout = WallDuration(milliseconds: 100)
        while running.load(ordering: .relaxed) && ros.isRunning.load(ordering: .relaxed) {
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
