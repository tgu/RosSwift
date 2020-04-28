import XCTest
import Foundation
@testable import msgbuilderLib

//FIXME:
let rosDistSource = "/Users/tgu/ros-install-osx/melodic_desktop_full_ws"

class GenTest: XCTestCase {

    override func setUp() {
    }

    override func tearDown() {
    }

    func testLoadMsgString() {
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


        if let spec = MsgSpec(text: logmsg, full_name: "rosgraph_msgs/Log") {
            XCTAssertEqual(spec.constants.count, 5)
            let names = spec.variables.map { $0.name }
            XCTAssertEqual(names, ["header","level","name","msg","file","function","line","topics"])
            let types = spec.variables.map { $0.field_type }
            XCTAssertEqual(types, ["std_msgs/Header","byte","string","string","string","string","uint32","string[]"])
        } else {
            XCTFail("'rosgraph_msgs/Log' not found")
        }
    }

    func testLoadMsgString2() {
        let logmsg = """
            Vector3 translation
            Quaternion rotation
            """

        let context = MsgContext()
        let search_path = ["std_msgs": ["\(rosDistSource)/src/std_msgs/msg"],
                           "geometry_msgs": ["\(rosDistSource)/src/common_msgs/geometry_msgs/msg"]]

        guard let spec = MsgSpec(text: logmsg, full_name: "geometry_msgs/Accel") else {
            XCTFail()
            return
        }
        XCTAssertEqual(spec.constants.count, 0)
        let names = spec.variables.map { $0.name }
        XCTAssertEqual(names, ["translation","rotation"])
        let types = spec.variables.map { $0.field_type }.sorted()
        XCTAssertEqual(types, ["geometry_msgs/Quaternion", "geometry_msgs/Vector3"])

        let specs = context.load_msg_depends(spec: spec, search_path: search_path)
        let deps = specs.map { $0.full_name }.sorted()
        XCTAssertEqual(deps,types)

        XCTAssertEqual(spec.compute_md5(msg_context: context), "ac9eff44abf714214112b05d54a3cf9b")


    }

    func testLoadFromFile() {
        let path = "\(rosDistSource)/src/common_msgs/geometry_msgs/msg/Accel.msg"
        let context = MsgContext()
        let spec = context.loadMsg(from: path, full_name: "geometry_msgs/Accel")
        XCTAssertEqual(spec?.constants.count, 0)
        let names = spec?.variables.map { $0.name }
        XCTAssertEqual(names, ["linear","angular"])
        let types = spec?.variables.map { $0.field_type }
        XCTAssertEqual(types, ["geometry_msgs/Vector3","geometry_msgs/Vector3"])
    }

    func testGenMD5() {

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
            XCTAssertNotNil(md5)
            XCTAssertEqual(md5, "acffd30cd6b6de30f120938c17c593fb")
        }

        let transf = """
            4a842b65f413084dc2b10fb484ea7f17 translation
            a779879fadf0160734f906b8c19c7004 rotation
            """


        do {
            let md5 = transf.hashed()
            XCTAssertNotNil(md5)
            XCTAssertEqual(md5, "ac9eff44abf714214112b05d54a3cf9b")
        }


    }

}
