//
//  NameRemappingWithNamespace.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-10-31.
//

import Testing
@testable import RosSwift
import rosmaster
import RosNetwork

class NameRemappingWithNamespace {
    let master: rosmaster.Master
    var remap: [String:String] {
        ["__master": master.address]
    }
    var host: String {
        master.host
    }

    init() throws {
        let network = RosNetwork(remappings: [:])
        master = rosmaster.Master(host: network.gHost, port: 11311, advertise: false)
        _ = try master.start().wait()
    }

    deinit {
        _ = master.stop()
    }

    @Test func testParameterRemapping() async throws {
        let args = CommandLine.arguments + ["/a/test_full:=/b/test_full",
                                            "/a/test_local:=test_local2",
                                            "test_relative:=/b/test_relative",
                                            "__ns:=a",
                                            "__master:=\(remap["__master"]!)"]

        let ros = try Ros(argv: args, name: "testParameterRemapping")
        await ros.param.set(key: "/b/test_full", value: "asdf")
        await ros.param.set(key: "/a/test_local2", value: "asdf")
        await ros.param.set(key: "/b/test_relative", value: "asdf")

        var param = ""

        #expect(ros.resolve(name: "test_full") == "/b/test_full")
        #expect(await ros.param.get("test_full", &param))
        #expect(ros.resolve(name: "/a/test_full") == "/b/test_full")
        #expect(await ros.param.get("/a/test_full", &param))

        #expect(ros.resolve(name: "test_local") == "/a/test_local2")
        #expect(await ros.param.get("test_local", &param))
        #expect(ros.resolve(name: "/a/test_local") == "/a/test_local2")
        #expect(await ros.param.get("/a/test_local", &param))

        #expect(ros.resolve(name: "test_relative") == "/b/test_relative")
        #expect(await ros.param.get("test_relative", &param))
        #expect(ros.resolve(name: "/a/test_relative") == "/b/test_relative")
        #expect(await ros.param.get("/a/test_relative", &param))

        #expect(ros.resolve(name: "1244") == nil)
        #expect(ros.resolve(name: "åäö") == "/a/åäö")

        let node_name = ros.name
        #expect(node_name == "/a/testParameterRemapping")

    }

}
