//
//  SpinnerMonitor.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-11-22.
//

import Foundation

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

class SpinnerMonitor {

    /// store spinner information per callback queue:
    /// Only alike spinners (single-threaded or multi-threaded) are allowed on a callback queue.
    /// For single-threaded spinners we store their thread id.
    /// We store the number of alike spinners operating on the callback queue.
    struct Entry {

        /// proper thread id of single-threaded spinner
        let tid: Thread?

        /// number of (alike) spinners serving this queue
        var num: Int

        init(threadID: Thread?) {
            self.tid = threadID
            self.num = 0
        }
    }

    static var main = SpinnerMonitor()

    private init() {}

    var spinningQueues = [ObjectIdentifier: Entry]()
    let mutex = DispatchQueue(label: "mutex")

    /// add a queue to the list
    func add(queue: CallbackQueue, singleThreaded: Bool) -> Bool {
        var thread: Thread?  // current thread id for single-threaded spinners, nil for multi-threaded ones
        if singleThreaded {
            thread = Thread.current
        }

        var success = true

        mutex.sync {
            if let entry = spinningQueues[ObjectIdentifier(queue)] {
                // spinner must be alike (all multi-threaded: 0, or single-threaded on same thread id)
                if entry.tid == thread {
                    spinningQueues[ObjectIdentifier(queue)]?.num += 1
                } else {
                    success = false
                }
            } else {
                // we will spin on any new queue
                var newEntry = Entry(threadID: thread)
                newEntry.num += 1
                spinningQueues[ObjectIdentifier(queue)] = newEntry
            }
        }

        return success
    }


    /// remove a queue from the list

    func remove(queue: CallbackQueue) {
        mutex.sync {
            guard let entry = spinningQueues[ObjectIdentifier(queue)] else {
                fatalError("Call to SpinnerMonitor::remove() without matching call to add().")
            }

            if let tid = entry.tid, tid != Thread.current {
                print("SpinnerMonitor::remove() called from different thread than add().")
            }

            guard entry.num > 0 else {
                fatalError("Call to SpinnerMonitor::remove() without matching call to add().")
            }
            spinningQueues[ObjectIdentifier(queue)]?.num -= 1
            if spinningQueues[ObjectIdentifier(queue)]?.num == 0 {
                spinningQueues.removeValue(forKey: ObjectIdentifier(queue))
            }
        }
    }
}

let DEFAULT_ERROR_MESSAGE = "Attempt to spin a callback queue from two spinners, one of them being single-threaded."
