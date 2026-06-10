//
//  connectionTests.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-10-23.
//

import Testing
@testable import RosSwift
@testable import StdMsgs
@testable import BinaryCoder
@testable import RosTime
import RosNetwork
import rosmaster
import Atomics
import Synchronization

@Suite("Connection tests", .serialized)
class ConnectionTests {
    let master: rosmaster.Master
    var remap: [String: String] {
        ["__master": master.address]
    }
    var host: String {
        master.host
    }
    var port: Int {
        master.port
    }

    init() async throws {
        let network = RosNetwork(remappings: [:])
        // port 0 lets the OS pick a free port so parallel suites don't collide.
        master = rosmaster.Master(host: network.gHost, port: 0, advertise: false)
        _ = try await master.start().get()
    }

    deinit {
        _ = master.stop()
    }

    @Test func nodeHandleConstructionDestruction() async throws {
        try await withRos(master: host, port: port) { ros in
            do {
                #expect(!ros.isStarted)
                let n1 = await ros.createNode()
                #expect(ros.isStarted)
                #expect(await n1.refCount == 1)
                #expect(n1.gNodeStartedByNodeHandle)
                do {
                    let n2 = await ros.createNode()
                    #expect(ros.isStarted)
                    #expect(await n2.refCount == 2)
                    #expect(!n2.gNodeStartedByNodeHandle)
                    do {
                        let n3 = await ros.createNode()
                        #expect(ros.isStarted)
                        #expect(await n3.refCount == 3)
                        #expect(!n3.gNodeStartedByNodeHandle)
                        do {
                            let n4 = await ros.createNode()
                            #expect(ros.isStarted)
                            #expect(!n4.gNodeStartedByNodeHandle)
                            #expect(await n4.refCount == 4)
                        }
                        #expect(await n1.refCount == 3)
                    }
                    #expect(await n1.refCount == 2)
                }
                #expect(await n1.refCount == 1)
                #expect(ros.isStarted)
            }
            #expect(ros.nodeReferenceCount.load(ordering: .relaxed) == 0)
            #expect(!ros.isStarted)
            do {
                let n5 = await ros.createNode()
                #expect(ros.isStarted)
                #expect(await n5.refCount == 1)
                #expect(n5.gNodeStartedByNodeHandle)
            }
            #expect(ros.nodeReferenceCount.load(ordering: .relaxed) == 0)
            #expect(!ros.isStarted)
        }
    }

    @Test func intraprocess() async throws {
        try await withRos(master: host, port: port) { ros in
            let _chatter: Mutex<Float64> = Mutex(0.0)
            var chatter: Float64 {
                _chatter.withLock { $0 }
            }
            let n = await ros.createNode()
            let chatter_pub = try #require(await n.advertise(topic: "/intrachatter", message: std_msgs.float64.self))

            @Sendable func chatterCallback(msg: std_msgs.float64) {
                print("I saw: [\(msg)]")
                _chatter.withLock { $0 = msg.data }
            }

            let vab = await n.subscribe(topic: "/intrachatter", queueSize: 1, callback: chatterCallback)
            #expect(vab != nil)

            await chatter_pub.publish(message: std_msgs.float64(10.0))
            try? await Task.sleep(for: .milliseconds(100))
            await ros.spinOnce()
            try? await Task.sleep(for: .milliseconds(100))

            #expect(chatter == 10.0)
        }
    }

    @Test func getPublishedTopics() async throws {
        try await withRos(master: host, port: port) { ros in
            let n = await ros.createNode()
            let advertised_topics = (1...8).map { "/test_topic_\($0)" }

            var pubs = [Publisher]()
            for adv_it in advertised_topics {
                if let pub = await n.advertise(topic: adv_it, message: std_msgs.float64.self) {
                    pubs.append(pub)
                } else {
                    Issue.record("Failed to advertise \(adv_it)")
                }
            }

            let topics = try await ros.master.getTopics(callerId: ros.name)

            for topic in advertised_topics {
                #expect(topics.contains { $0.name == topic })
            }
        }
    }

    @Test func trackedObjectWithAdvertiseSubscriberCallback() async throws {
        try await withRos(name: "testCallServiceProvider", master: host, port: port) { rosProvider in
            let g_recv_count = ManagedAtomic(0)
            let g_sub_count = ManagedAtomic(0)

            @Sendable func connectCallback(_ publisher: SingleSubscriberPublisher) {
                g_sub_count.wrappingIncrement(ordering: .relaxed)
            }

            @Sendable func subscriberCallback(req: String) {
                g_recv_count.wrappingIncrement(ordering: .relaxed)
            }

            let n = await rosProvider.createNode()

            final class Trackable: TrackableObject {
                let id: String = "rer"
            }

            var tracked: Trackable? = Trackable()

            let pub = try #require(await n.advertise(topic: "/test", queueSize: 0, connectCall: connectCallback, message: std_msgs.string.self, tracked_object: tracked))

            var sub = await n.subscribe(topic: "/test", queueSize: 1, callback: subscriberCallback)
            await pub.publish(message: "hello")

            await rosProvider.spinOnce()

            let d = WallDuration(seconds: 0.01)
            while g_sub_count.load(ordering: .relaxed) == 0 {
                await d.sleep()
                await rosProvider.spinOnce()
            }

            #expect(g_sub_count.load(ordering: .relaxed) == 1)

            sub?.shutdown()

            tracked = nil

            await pub.publish(message: "hello")

            await rosProvider.spinOnce()

            sub = await n.subscribe(topic: "/test", queueSize: 1, callback: subscriberCallback)

            for _ in 0..<10 {
                await d.sleep()
                await rosProvider.spinOnce()
            }

            #expect(g_sub_count.load(ordering: .relaxed) == 1)
            #expect(g_recv_count.load(ordering: .relaxed) == 1)
        }
    }

    @Test func trackedObjectWithSubscriptionCallback() async throws {
        try await withRos(name: "testCallServiceProvider", master: host, port: port) { rosProvider in
            let g_recv_count = ManagedAtomic(0)

            @Sendable func subscriberCallback(req: String) {
                g_recv_count.wrappingIncrement(ordering: .relaxed)
            }

            let n = await rosProvider.createNode()

            final class Trackable: TrackableObject {
                let id: String = "rer"
            }

            var tracked: Trackable? = Trackable()

            let sub = await n.subscribe(topic: "/test", queueSize: 1, callback: subscriberCallback, tracked_object: tracked)

            let pub = try #require(await n.advertise(topic: "/test", queueSize: 0, message: String.self))

            let d = WallDuration(seconds: 0.01)
            while g_recv_count.load(ordering: .relaxed) == 0 {
                await pub.publish(message: "hello")
                await d.sleep()
                await rosProvider.spinOnce()
            }
            #expect(g_recv_count.load(ordering: .relaxed) == 1)

            tracked = nil

            await pub.publish(message: "hello")
            for _ in 0..<10 {
                await d.sleep()
                await rosProvider.spinOnce()
            }

            #expect(g_recv_count.load(ordering: .relaxed) == 1)

            sub?.shutdown()
        }
    }

    @Test func nodeHandleParentWithRemappings() async throws {
        try await withRos(master: host, port: port) { ros in
            let remappings = ["a": "b", "c": "d"]
            let n1 = try #require(await ros.createNode(ns: "/", remappings: remappings))

            #expect(await n1.namespace == "/")
            #expect(await n1.remappings == ["/a": "/b", "/c": "/d"])

            #expect(await n1.resolveName(name: "a") == "/b")
            #expect(await n1.resolveName(name: "/a") == "/b")
            #expect(await n1.resolveName(name: "c") == "/d")
            #expect(await n1.resolveName(name: "/c") == "/d")

            let n2 = await ros.createNode(parent: n1, ns: "my_ns")

            #expect(await n2.resolveName(name: "a") == "/my_ns/a")
            #expect(await n2.resolveName(name: "/a") == "/b")
            #expect(await n2.resolveName(name: "c") == "/my_ns/c")
            #expect(await n2.resolveName(name: "/c") == "/d")

            let n3 = await ros.createNode(parent: n2)

            #expect(await n3.resolveName(name: "a") == "/my_ns/a")
            #expect(await n3.resolveName(name: "/a") == "/b")
            #expect(await n3.resolveName(name: "c") == "/my_ns/c")
            #expect(await n3.resolveName(name: "/c") == "/d")
        }
    }

    final class SubscribeHelper: Sendable {
        let recv_count_ = ManagedAtomic(0)

        @Sendable func callback(_ f: std_msgs.float64) {
            recv_count_.wrappingIncrement(ordering: .relaxed)
        }
    }

    @Test func subscriberDestructionMultipleCallbacks() async throws {
        try await withRos(master: host, port: port) { ros in
            let n = await ros.createNode()
            let pub = try #require(await n.advertise(topic: "test", message: std_msgs.float64.self))
            let msg = std_msgs.float64(3.14)

            let g_recv_count = ManagedAtomic(0)
            @Sendable func subscriberCallback(_ f: std_msgs.float64) {
                g_recv_count.wrappingIncrement(ordering: .relaxed)
            }

            do {
                let helper = SubscribeHelper()
                let sub_class = await n.subscribe(topic: "test", queueSize: 0, callback: helper.callback)
                #expect(sub_class != nil)
                let d = RosDuration(milliseconds: 50)
                var last_class_count = helper.recv_count_.load(ordering: .relaxed)
                while last_class_count == helper.recv_count_.load(ordering: .relaxed) {
                    await pub.publish(message: msg)
                    await ros.spinOnce()
                    await d.sleep()
                }

                var last_fn_count = g_recv_count.load(ordering: .relaxed)
                do {
                    let sub_fn = await n.subscribe(topic: "test", queueSize: 0, callback: subscriberCallback)
                    #expect(sub_fn != nil)
                    last_fn_count = g_recv_count.load(ordering: .relaxed)
                    while last_fn_count == g_recv_count.load(ordering: .relaxed) {
                        await pub.publish(message: msg)
                        await ros.spinOnce()
                        await d.sleep()
                    }
                }

                last_fn_count = g_recv_count.load(ordering: .relaxed)
                last_class_count = helper.recv_count_.load(ordering: .relaxed)
                while last_class_count == helper.recv_count_.load(ordering: .relaxed) {
                    await pub.publish(message: msg)
                    await ros.spinOnce()
                    await d.sleep()
                }

                #expect(last_fn_count == g_recv_count.load(ordering: .relaxed))
            }
        }
    }

    @Test func publisherMultiple() async throws {
        try await withRos(master: host, port: port) { ros in
            do {
                let n = await ros.createNode()
                let pub1 = await n.advertise(topic: "/test", message: std_msgs.float64.self)
                #expect(pub1 != nil)

                do {
                    let pub2 = await n.advertise(topic: "/test", message: std_msgs.float64.self)
                    #expect(pub2 != nil)

                    let topics1 = ros.getAdvertisedTopics()
                    let t1 = topics1.first(where: { $0 == "/test" })
                    #expect(t1 != nil)
                }

                let topics2 = ros.getAdvertisedTopics()
                let t2 = topics2.first(where: { $0 == "/test" })
                #expect(t2 != nil)
            }
            await WallDuration(milliseconds: 100).sleep()
            print("leaving scope")
            let topics3 = ros.getAdvertisedTopics()
            print(topics3)
            let t3 = topics3.first(where: { $0 == "/test" })
            #expect(t3 == nil)
        }
    }

    @Test func publisherCallback() async throws {
        try await withRos(master: host, port: port) { ros in
            let n = await ros.createNode()
            let conns = ManagedAtomic(0)
            let disconns = ManagedAtomic(0)

            let ops = AdvertiseOptions(
                topic: "/testCallback",
                queueSize: 1,
                latch: false,
                std_msgs.int8.self,
                connectCall: { _ in
                    conns.wrappingIncrement(ordering: .relaxed)
                },
                disconnectCall: { _ in
                    disconns.wrappingIncrement(ordering: .relaxed)
                }
            )

            let pub = await n.advertise(ops: ops)
            do {
                let sub = await n.subscribe(topic: "/testCallback") { (msg: Int8) -> Void in print("msg = \(msg)") }
                #expect(sub?.publisherCount == 1)
                #expect(conns.load(ordering: .relaxed) == 1)
                #expect(disconns.load(ordering: .relaxed) == 0)
                #expect(pub?.numSubscribers == 1)
            }

            #expect(pub?.numSubscribers == 0)
            #expect(conns.load(ordering: .relaxed) == 1)
            #expect(disconns.load(ordering: .relaxed) == 1)
        }
    }

    @Test func multipleRos() async throws {
        let r1 = try Ros(name: "testMultipleRos", namespace: "ett", remappings: remap)
        let r2 = try Ros(name: "testMultipleRos", namespace: "två", remappings: remap)

        let n1 = await r1.createNode()
        let p1 = await n1.advertise(topic: "test", message: Int64.self)

        let n2 = await r2.createNode()

        let received = ManagedAtomic<Int64>(0)

        let s2 = await n2.subscribe(topic: "/ett/test", queueSize: 100) { (msg: Int64) -> Void in
            received.wrappingIncrement(ordering: .relaxed)
        }
        #expect(s2 != nil)

        Task.detached {
            await r2.spin()
        }

        await WallDuration(milliseconds: 100).sleep()

        let start = WallTime.now
        let sent: Int64 = 100
        for i in 0..<sent {
            await p1?.publish(message: i)
        }
        let end = WallTime.now
        print("elapsed time = \((end - start).toSec()) ")
        await WallDuration(milliseconds: 1000).sleep()

        #expect(received.load(ordering: .relaxed) == sent)

        await r1.shutdownAsync()
        await r2.shutdownAsync()
    }

    @Test func `internal`() async throws {
        try await withRos(master: host, port: port) { ros in
            let n = await ros.createNode()
            let p = await n.advertise(topic: "/testInternal", message: Int64.self)

            let received = ManagedAtomic<Int64>(0)

            let s = await n.subscribe(topic: "/testInternal", queueSize: 10) { (msg: Int64) -> Void in
                #expect(received.load(ordering: .relaxed) == msg)
                received.wrappingIncrement(ordering: .relaxed)
            }
            #expect(s != nil)

            Task.detached(priority: .high) {
                await ros.spin()
            }

            await WallDuration(milliseconds: 100).sleep()

            let start = WallTime.now
            let sent: Int64 = 100
            for i in 0..<sent {
                await p?.publish(message: i)
                await WallDuration(milliseconds: 10).sleep()
            }
            let end = WallTime.now
            print("elapsed time = \((end - start).toSec()) ")
            await WallDuration(milliseconds: 100).sleep()

            #expect(received.load(ordering: .relaxed) == sent)
        }
    }

    @Test func nonLatching() async throws {
        try await withRos(master: host, port: port) { ros in
            let n = await ros.createNode()

            let pub = try #require(await n.advertise(topic: "/testInternal", latch: false, message: Int64.self))
            await pub.publish(message: Int64(1))

            let count = ManagedAtomic(0)
            _ = await n.subscribe(topic: "/testInternal") { (msg: Int64) -> Void in
                _ = count.loadThenWrappingIncrement(ordering: .relaxed)
            }

            await ros.spinOnce()

            #expect(count.load(ordering: .relaxed) == 0)
        }
    }

    @Test func latching() async throws {
        try await withRos(master: host, port: port) { ros in
            let n = await ros.createNode()

            let pub = try #require(await n.advertise(topic: "/testInternal", latch: true, message: Int64.self))
            await pub.publish(message: Int64(1))

            let count = ManagedAtomic(0)
            _ = await n.subscribe(topic: "/testInternal") { (msg: Int64) -> Void in
                _ = count.loadThenWrappingIncrement(ordering: .relaxed)
            }

            await ros.spinOnce()

            #expect(count.load(ordering: .relaxed) == 1)
        }
    }

    @Test func latchingMultipleSubscribers() async throws {
        try await withRos(master: host, port: port) { ros in
            let n = await ros.createNode()

            let pub = try #require(await n.advertise(topic: "/testInternal", latch: true, message: Int64.self))
            await pub.publish(message: Int64(1))

            let count1 = ManagedAtomic(0)
            let count2 = ManagedAtomic(0)

            _ = await n.subscribe(topic: "/testInternal") { (msg: Int64) -> Void in
                _ = count1.loadThenWrappingIncrement(ordering: .relaxed)
            }

            await ros.spinOnce()

            #expect(count1.load(ordering: .relaxed) == 1)
            #expect(count2.load(ordering: .relaxed) == 0)

            _ = await n.subscribe(topic: "/testInternal") { (msg: Int64) -> Void in
                _ = count2.loadThenWrappingIncrement(ordering: .relaxed)
            }

            await Task.yield()
            await ros.spinOnce()

            #expect(count1.load(ordering: .relaxed) == 1)
            #expect(count2.load(ordering: .relaxed) == 1)
        }
    }
}
