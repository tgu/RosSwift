//
//  connectionTests.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-10-23.
//

import XCTest
@testable import RosSwift
@testable import StdMsgs
@testable import BinaryCoder
@testable import RosTime
import RosNetwork
import rosmaster
import Atomics

class connectionTests: RosTest {

    func testNodeHandleConstructionDestruction() {

        let ros = Ros(master: host)

        do {
            XCTAssertFalse(ros.isStarted.load(ordering: .relaxed))
            let n1 = ros.createNode()
            XCTAssert(ros.isStarted.load(ordering: .relaxed))
            XCTAssertEqual(n1.refCount,1)
            XCTAssert(n1.gNodeStartedByNodeHandle)
            do {
                let n2 = ros.createNode()
                XCTAssert(ros.isStarted.load(ordering: .relaxed))
                XCTAssertEqual(n2.refCount,2)
                XCTAssertFalse(n2.gNodeStartedByNodeHandle)
                do {
                    let n3 = ros.createNode()
                    XCTAssert(ros.isStarted.load(ordering: .relaxed))
                    XCTAssertEqual(n3.refCount,3)
                    XCTAssertFalse(n3.gNodeStartedByNodeHandle)
                    do {
                        let n4 = ros.createNode()
                        XCTAssert(ros.isStarted.load(ordering: .relaxed))
                        XCTAssertFalse(n4.gNodeStartedByNodeHandle)
                        XCTAssertEqual(n4.refCount,4)
                    }
                    XCTAssertEqual(n1.refCount,3)
                }
                XCTAssertEqual(n1.refCount,2)
            }
            XCTAssertEqual(n1.refCount,1)
            XCTAssert(ros.isStarted.load(ordering: .relaxed))
        }
        XCTAssertEqual(ros.nodeReferenceCount.load(ordering: .relaxed), 0)
        XCTAssertFalse(ros.isStarted.load(ordering: .relaxed))
        do {
            let n5 = ros.createNode()
            XCTAssert(ros.isStarted.load(ordering: .relaxed))
            XCTAssertEqual(n5.refCount,1)
            XCTAssert(n5.gNodeStartedByNodeHandle)
        }
        XCTAssertEqual(ros.nodeReferenceCount.load(ordering: .relaxed), 0)
        XCTAssertFalse(ros.isStarted.load(ordering: .relaxed))
        ros.shutdown()
    }


    func testIntraprocess() {
        let ros = Ros(master: host)
        var chatter : Float64 = 0.0
        let n = ros.createNode()
        guard let chatter_pub = n.advertise(topic: "/intrachatter", message: std_msgs.float64.self ) else {
            exit(1)
        }

        func chatterCallback(msg: std_msgs.float64) {
            print("I saw: [\(msg)]")
            chatter = msg.data
        }
        let vab = n.subscribe(topic: "/intrachatter", queueSize: 1, callback: chatterCallback)
        XCTAssertNotNil(vab)

        chatter_pub.publish(message: std_msgs.float64(10.0))
        usleep(100000)
        ros.spinOnce()
        usleep(100000)

        XCTAssertEqual(chatter, 10.0)

    }

    func testgetPublishedTopics() {
        let ros = Ros(master: host)
        let n = ros.createNode()
        let advertised_topics = (1...8).map { "/test_topic_\($0)" }

        var pubs = [Publisher]()
        for adv_it in advertised_topics {
            if let pub = n.advertise(topic: adv_it, message: std_msgs.float64.self) {
                pubs.append(pub)
            } else {
                XCTFail(adv_it)
            }
        }

        guard let topics = try? ros.master.getTopics(callerId: ros.name).wait() else {
            XCTFail()
            return
        }

        for topic in advertised_topics {
            XCTAssert(topics.contains { $0.name == topic })
        }
    }

    func testnodeHandleParentWithRemappings() {
        let ros = Ros(master: host)
        let remappings = ["a":"b", "c":"d"]
        guard let n1 = ros.createNode(ns: "/", remappings: remappings) else {
            XCTFail()
            return
        }

        
        XCTAssertEqual(n1.namespace, "/")
        XCTAssertEqual(n1.remappings, ["/a":"/b","/c":"/d"])


        // Sanity checks

        XCTAssertEqual(n1.resolveName(name: "a"), "/b")
        XCTAssertEqual(n1.resolveName(name: "/a"), "/b")
        XCTAssertEqual(n1.resolveName(name: "c"), "/d")
        XCTAssertEqual(n1.resolveName(name: "/c"), "/d")

        let n2 = ros.createNode(parent: n1, ns: "my_ns")

        XCTAssertEqual(n2.resolveName(name: "a"), "/my_ns/a")
        XCTAssertEqual(n2.resolveName(name: "/a"), "/b")
        XCTAssertEqual(n2.resolveName(name: "c"), "/my_ns/c")
        XCTAssertEqual(n2.resolveName(name: "/c"), "/d")

        let n3 = ros.createNode(parent: n2)

        XCTAssertEqual(n3.resolveName(name: "a"), "/my_ns/a")
        XCTAssertEqual(n3.resolveName(name: "/a"), "/b")
        XCTAssertEqual(n3.resolveName(name: "c"), "/my_ns/c")
        XCTAssertEqual(n3.resolveName(name: "/c"), "/d")

    }

    var g_recv_count: Int32 = 0
    func subscriberCallback(_ f: std_msgs.float64) {
        g_recv_count = g_recv_count + 1
    }


    class SubscribeHelper {
        let recv_count_ = ManagedAtomic(0)

        func callback(_ f: std_msgs.float64) {
            recv_count_.wrappingIncrement(ordering: .relaxed)
        }
    }


    func testSubscriberDestructionMultipleCallbacks() {
        let ros = Ros(master: host)
        let n = ros.createNode()
        guard let pub = n.advertise(topic: "test", message: std_msgs.float64.self) else {
            XCTFail()
            return
        }
        let msg = std_msgs.float64(3.14)
        do {
            let helper = SubscribeHelper()
            let sub_class = n.subscribe(topic: "test", queueSize: 0, callback: helper.callback)
            XCTAssertNotNil(sub_class)
            let d = RosDuration(milliseconds: 50)
            var last_class_count = helper.recv_count_.load(ordering: .relaxed)
            while last_class_count == helper.recv_count_.load(ordering: .relaxed) {
                pub.publish(message: msg)
                ros.spinOnce()
                d.sleep()
            }

            var last_fn_count = g_recv_count
            do {
                let sub_fn = n.subscribe(topic: "test", queueSize: 0, callback: subscriberCallback)
                XCTAssertNotNil(sub_fn)
                last_fn_count = g_recv_count
                while last_fn_count == g_recv_count {
                    pub.publish(message: msg)
                    ros.spinOnce()
                    d.sleep()
                }
            }

            last_fn_count = g_recv_count
            last_class_count = helper.recv_count_.load(ordering: .relaxed)
            while last_class_count == helper.recv_count_.load(ordering: .relaxed) {
                pub.publish(message: msg)
                ros.spinOnce()
                d.sleep()
            }

            XCTAssertEqual(last_fn_count, g_recv_count);
        }
    }

    func testPublisherMultiple() {
        let ros = Ros(master: host)

        do {
            let n = ros.createNode()
            let pub1 = n.advertise(topic: "/test", message: std_msgs.float64.self)
            XCTAssertNotNil(pub1)
            
            do {
                let pub2 = n.advertise(topic: "/test", message: std_msgs.float64.self)
                XCTAssertNotNil(pub2)

                let topics1 = ros.getAdvertisedTopics()
                let t1 = topics1.first(where: { $0 == "/test" })
                XCTAssertNotNil(t1)
            }

            let topics2 = ros.getAdvertisedTopics()
            let t2 = topics2.first(where: { $0 == "/test" })
            XCTAssertNotNil(t2)
       }
        WallDuration(milliseconds: 100).sleep()
        print("leaving scope")
        let topics3 = ros.getAdvertisedTopics()
        print(topics3)
        let t3 = topics3.first(where: { $0 == "/test" })
        XCTAssertNil(t3)
    }

    func testPublisherCallback() {
        let ros = Ros(master: host)

        let n = ros.createNode()
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
        
        let pub = n.advertise(ops: ops)
        do {
            let sub = n.subscribe(topic: "/testCallback") { (msg: Int8) -> Void  in }
            XCTAssertEqual(sub?.publisherCount, 1)
            XCTAssertEqual(conns.load(ordering: .relaxed), 1)
            XCTAssertEqual(disconns.load(ordering: .relaxed), 0)
            XCTAssertEqual(pub?.numSubscribers, 1)
        }

        XCTAssertEqual(pub?.numSubscribers, 0)
        XCTAssertEqual(conns.load(ordering: .relaxed), 1)
        XCTAssertEqual(disconns.load(ordering: .relaxed), 1)

    }



    func testMultipleRos() {
        let r1 = Ros(name: "testMultipleRos", namespace: "ett", remappings: remap)
        let r2 = Ros(name: "testMultipleRos", namespace: "två", remappings: remap)

        let n1 = r1.createNode()
        let p1 = n1.advertise(topic: "test", message: Int64.self)

        let n2 = r2.createNode()

        let received = ManagedAtomic<Int64>(0)

        let s2 = n2.subscribe(topic: "/ett/test", queueSize: 100) { (msg: Int64) -> Void in
            received.wrappingIncrement(ordering: .relaxed)
        }
        XCTAssertNotNil(s2)

        DispatchQueue(label: "r1").async {
            r2.spin()
        }

        WallDuration(milliseconds: 100).sleep()

        let start = WallTime.now
        let sent:Int64 = 100
        for i in 0..<sent {
            p1?.publish(message: i)
        }
        let end = WallTime.now
        print("elapsed time = \((end-start).toSec()) ")
        WallDuration(milliseconds: 1000).sleep()

        XCTAssertEqual(received.load(ordering: .relaxed), sent)
    }

    func testInternal() {
        let ros = Ros(master: host)
        let n = ros.createNode()
        let p = n.advertise(topic: "/testInternal", message: Int64.self)

        let received = ManagedAtomic<Int64>(0)

        let s = n.subscribe(topic: "/testInternal", queueSize: 10) { (msg: Int64) -> Void in
            received.wrappingIncrement(ordering: .relaxed)
        }
        XCTAssertNotNil(s)

        DispatchQueue(label: "r").async {
            ros.spin()
        }

        WallDuration(milliseconds: 100).sleep()

        let start = WallTime.now
        let sent:Int64 = 100
        for i in 0..<sent {
            p?.publish(message: i)
        }
        let end = WallTime.now
        print("elapsed time = \((end-start).toSec()) ")
        WallDuration(milliseconds: 100).sleep()

        XCTAssertEqual(received.load(ordering: .relaxed), sent)
    }
    
    func testNonLatching() {
        let ros = Ros(master: host)

        let n = ros.createNode()
        
        let pub = n.advertise(topic: "/testInternal", latch: false, message: Int64.self)!
        pub.publish(message: Int64(1))
        
        var count = 0
        _ = n.subscribe(topic: "/testInternal") { (msg: Int64) -> Void in
            count += 1
        }
        
        ros.spinOnce()

        XCTAssertEqual(count, 0)

    }
    

    
    func testLatching() {
        let ros = Ros(master: host)

        let n = ros.createNode()
        
        let pub = n.advertise(topic: "/testInternal", latch: true, message: Int64.self)!
        pub.publish(message: Int64(1))
        
        var count = 0
        _ = n.subscribe(topic: "/testInternal") { (msg: Int64) -> Void in
            count += 1
        }
        
        ros.spinOnce()

        XCTAssertEqual(count, 1)

    }
    
    func testLatchingMultipleSubscribers() {
        let ros = Ros(master: host)

        let n = ros.createNode()
        
        let pub = n.advertise(topic: "/testInternal", latch: true, message: Int64.self)!
        pub.publish(message: Int64(1))
        
        var count1 = 0
        var count2 = 0
        
        _ = n.subscribe(topic: "/testInternal") { (msg: Int64) -> Void in
            count1 += 1
        }
        
        ros.spinOnce()

        XCTAssertEqual(count1, 1)
        XCTAssertEqual(count2, 0)
        
        _ = n.subscribe(topic: "/testInternal") { (msg: Int64) -> Void in
            count2 += 1
        }
        
        ros.spinOnce()

        XCTAssertEqual(count1, 1)
        XCTAssertEqual(count1, 1)


    }




}
