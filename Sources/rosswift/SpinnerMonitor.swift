//
//  SpinnerMonitor.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-11-22.
//

import Foundation
import Synchronization

/// Class to monitor running single-threaded spinners.
///
///  Calling the callbacks of a callback queue _in order_, requires a unique SingleThreadedSpinner
///  spinning on the queue. Other threads accessing the callback queue will probably intercept execution order.
///
///  To avoid multiple SingleThreadedSpinners (started from different threads) to operate on the same callback queue,
///  this class stores a map of all spinned callback queues.
///  If the spinner is single threaded, the corresponding thread-id is stored in the map
///  and if other threads will try to spin the same queue, an error message is issued.
///
///  If the spinner is multi-threaded, the stored thread-id is NULL and future SingleThreadedSpinners
///  should not spin this queue. However, other multi-threaded spinners are allowed.

final class SpinnerMonitor: Sendable {
    
    /// store spinner information per callback queue:
    /// Only alike spinners (single-threaded or multi-threaded) are allowed on a callback queue.
    /// For single-threaded spinners we store their thread id.
    /// We store the number of alike spinners operating on the callback queue.
    struct Entry: Sendable {

        /// identity of the single-threaded spinner's thread (nil for
        /// multi-threaded). Stored as an `ObjectIdentifier` rather than the
        /// `Thread` itself so the entry is `Sendable` and can live inside the
        /// `Mutex`.
        let tid: ObjectIdentifier?

        /// number of (alike) spinners serving this queue
        var num: Int

        init(threadID: ObjectIdentifier?) {
            self.tid = threadID
            self.num = 0
        }
    }
    
    static let main = SpinnerMonitor()
    
    private init() {}
    
    // Internal mutable state is protected by a Mutex, which keeps the whole
    // monitor checked-`Sendable`.
    private let spinningQueues = Mutex<[ObjectIdentifier: Entry]>([:])

    /// add a queue to the list
    func add(queue: AsyncCallbackQueue, singleThreaded: Bool) -> Bool {
        // current thread identity for single-threaded spinners, nil for multi-threaded ones
        let thread: ObjectIdentifier? = singleThreaded ? ObjectIdentifier(Thread.current) : nil

        return spinningQueues.withLock { queues in
            let key = ObjectIdentifier(queue)
            if let entry = queues[key] {
                // spinner must be alike (all multi-threaded: 0, or single-threaded on same thread id)
                if entry.tid == thread {
                    queues[key]?.num += 1
                    return true
                } else {
                    return false
                }
            } else {
                // we will spin on any new queue
                var newEntry = Entry(threadID: thread)
                newEntry.num += 1
                queues[key] = newEntry
                return true
            }
        }
    }


    /// remove a queue from the list

    func remove(queue: AsyncCallbackQueue) {
        spinningQueues.withLock { queues in
            let key = ObjectIdentifier(queue)
            guard let entry = queues[key] else {
                fatalError("Call to SpinnerMonitor::remove() without matching call to add().")
            }

            if let tid = entry.tid, tid != ObjectIdentifier(Thread.current) {
                ROS_WARNING("SpinnerMonitor::remove() called from different thread than add().")
            }

            guard entry.num > 0 else {
                fatalError("Call to SpinnerMonitor::remove() without matching call to add().")
            }
            queues[key]?.num -= 1
            if queues[key]?.num == 0 {
                queues.removeValue(forKey: key)
            }
        }
    }
}

let DEFAULT_ERROR_MESSAGE = "Attempt to spin a callback queue from two spinners, one of them being single-threaded."
