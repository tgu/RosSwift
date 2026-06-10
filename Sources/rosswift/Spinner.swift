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
    
    func spin(ros: Ros, queue: AsyncCallbackQueue?) async
}

/// Spinner which runs in a single thread.

struct SingleThreadSpinner: Spinner {
    
    func spin(ros: Ros, queue: AsyncCallbackQueue? = nil) async {
        let useQueue = queue != nil ? queue! : ros.getGlobalCallbackQueue()
        if !SpinnerMonitor.main.add(queue: useQueue, singleThreaded: true) {
            fatalError("SingleThreadedSpinner: \(DEFAULT_ERROR_MESSAGE) You might want to use a MultiThreadedSpinner instead.")
        }
        
        let timeout = WallDuration(milliseconds: 100)
        while ros.isRunning.load(ordering: .relaxed) {
            do {
                try await useQueue.callAvailable(timeout: timeout)
            } catch let error {
                ROS_ERROR("\(error)")
            }
        }
        SpinnerMonitor.main.remove(queue: useQueue)
    }
}

final class AsyncSpinnner {
    private var threads = [Thread]()
    private let threadCount: Int
    private let callbackQueue: AsyncCallbackQueue
    private let running = ManagedAtomic(false)
    private let ros: Ros
    
    init(ros: Ros, threadCount: Int, queue: AsyncCallbackQueue? = nil) {
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
        
        fatalError("AsyncSpinnner.start() is not implemented")
    }
    
    func stop() {
        guard running.compareExchange(expected: true, desired: false, ordering: .relaxed).exchanged else {
            return
        }
        
        threads.removeAll()
        
        SpinnerMonitor.main.remove(queue: callbackQueue)
        
    }
    
    private func threadFunc() async {
        disableAllSignalsInThisThread()
        let queue = callbackQueue
        let useCallAvailable = threadCount == 1
        let timeout = WallDuration(milliseconds: 100)
        while running.load(ordering: .relaxed) && ros.isRunning.load(ordering: .relaxed) {
            do {
                if useCallAvailable {
                    try await queue.callAvailable(timeout: timeout)
                } else {
                    _ = try await queue.callOne(timeout: timeout)
                }
            } catch let error {
                ROS_ERROR(error.localizedDescription)
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
