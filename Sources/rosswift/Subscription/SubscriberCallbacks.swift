//
//  SubscriberCallbacks.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//


public typealias SubscriberStatusCallback = @Sendable (SingleSubscriberPublisher) -> Void

// `weak var trackedObject` blocks checked `Sendable` — weak references to
// classes are mutated implicitly by ARC, so the compiler can't prove
// thread-safety. The runtime guarantees atomic load/store of weak refs, so
// `@unchecked` is honest here.
final class SubscriberCallbacks: @unchecked Sendable {
    let connect: SubscriberStatusCallback?
    let disconnect: SubscriberStatusCallback?
    let hasTrackedObject: Bool
    weak var trackedObject: TrackableObject?
    
    init(connect: SubscriberStatusCallback?,
         disconnect: SubscriberStatusCallback?,
         hasTrackedObject: Bool,
         trackedObject: TrackableObject? = nil) {
        
        self.connect = connect
        self.disconnect = disconnect
        self.hasTrackedObject = hasTrackedObject
        self.trackedObject = trackedObject
    }
}
