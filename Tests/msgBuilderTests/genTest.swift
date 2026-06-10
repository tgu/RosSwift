import Testing
import Foundation
@testable import msgbuilderLib

let rosDistSource = ProcessInfo.processInfo.environment["ROS_DIST_SOURCE"] ?? ""

@Suite("Message builder tests")
struct GenTest {

    @Test func loadMsgString() throws {
        let logmsg = """
            ##
            ## Severity level constants
            ##
            byte DEBUG=1 #debug level
            byte INFO=2  #general level
            byte WARN=4  #warning level
            byte ERROR=8 #error level
            byte FATAL=16 #fatal/critical level
            ##
            ## Fields
            ##
            Header header
            byte level
            string name # name of the node
            string msg # message
            string file # file the message came from
            string function # function the message came from
            uint32 line # line the message came from
            string[] topics # topic names that the node publishes
            """

        let spec = try #require(MsgSpec(text: logmsg, full_name: "rosgraph_msgs/Log", messageType: .message, generate: true),
                                "'rosgraph_msgs/Log' not found")
        #expect(spec.constants.count == 5)
        let names = spec.variables.map { $0.name }
        #expect(names == ["header","level","name","msg","file","function","line","topics"])
        let types = spec.variables.map { $0.field_type }
        #expect(types == ["std_msgs/Header","byte","string","string","string","string","uint32","string[]"])
    }

    @Test func loadMsgString2() throws {
        if rosDistSource.isEmpty {
            return
        }

        let logmsg = """
            Vector3 translation
            Quaternion rotation
            """

        let context = MsgContext(useBuiltin: false)
        let search_path = ["std_msgs": ["\(rosDistSource)/src/std_msgs/msg"],
                           "geometry_msgs": ["\(rosDistSource)/src/common_msgs/geometry_msgs/msg"]]

        let spec = try #require(MsgSpec(text: logmsg, full_name: "geometry_msgs/Accel", messageType: .message, generate: true))
        #expect(spec.constants.count == 0)
        let names = spec.variables.map { $0.name }
        #expect(names == ["translation","rotation"])
        let types = spec.variables.map { $0.field_type }.sorted()
        #expect(types == ["geometry_msgs/Quaternion", "geometry_msgs/Vector3"])

        let specs = context.load_msg_depends(spec: spec, search_path: search_path)
        let deps = specs.map { $0.full_name }.sorted()
        #expect(deps == types)

        #expect(spec.compute_md5(msg_context: context) == "ac9eff44abf714214112b05d54a3cf9b")
    }

    @Test func loadSrvFile() throws {
        if rosDistSource.isEmpty {
            return
        }
        let context = MsgContext(useBuiltin: false)
        let search_path = ["std_msgs": ["\(rosDistSource)/src/std_msgs/msg"],
                           "control_msgs": ["\(rosDistSource)/src/control_msgs/msg","\(rosDistSource)/src/control_msgs/srv"],
                            "geometry_msgs": ["\(rosDistSource)/src/common_msgs/geometry_msgs/msg"]]
        let srv = try #require(context.load_srv_by_type(srv_type: "control_msgs/QueryTrajectoryState", search_path: search_path))
        #expect(srv.request.full_name == "control_msgs/QueryTrajectoryStateRequest")
        #expect(srv.response.full_name == "control_msgs/QueryTrajectoryStateResponse")
    }

    @Test func loadFromFile() {
        if rosDistSource.isEmpty {
            return
        }
        let path = "\(rosDistSource)/src/common_msgs/geometry_msgs/msg/Accel.msg"
        let context = MsgContext(useBuiltin: false)
        let spec = context.loadMsg(from: path, full_name: "geometry_msgs/Accel") as? MsgSpec
        #expect(spec?.constants.count == 0)
        let names = spec?.variables.map { $0.name }
        #expect(names == ["linear","angular"])
        let types = spec?.variables.map { $0.field_type }
        #expect(types == ["geometry_msgs/Vector3","geometry_msgs/Vector3"])
    }

    @Test func genMD5() {
        let cleaned = """
            byte DEBUG=1
            byte INFO=2
            byte WARN=4
            byte ERROR=8
            byte FATAL=16
            2176decaecbce78abc3b96ef049fabed header
            byte level
            string name
            string msg
            string file
            string function
            uint32 line
            string[] topics
            """

        do {
            let md5 = cleaned.hashed()
            #expect(md5 != nil)
            #expect(md5 == "acffd30cd6b6de30f120938c17c593fb")
        }

        let transf = """
            4a842b65f413084dc2b10fb484ea7f17 translation
            a779879fadf0160734f906b8c19c7004 rotation
            """

        do {
            let md5 = transf.hashed()
            #expect(md5 != nil)
            #expect(md5 == "ac9eff44abf714214112b05d54a3cf9b")
        }
    }

    @Test func md5() {
        let text = "953b798c0f514ff060a53a3498ce6246 pose".hashed()
        #expect(text == "4f3e0bbe7a24e1f929488cd1970222d3")

        let t3 = "953b798c0f514ff060a53a3498ce6246 pose".md5
        #expect(t3 == "4f3e0bbe7a24e1f929488cd1970222d3")
    }
}
