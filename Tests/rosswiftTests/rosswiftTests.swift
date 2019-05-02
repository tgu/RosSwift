import XCTest
@testable import RosSwift
@testable import StdMsgs
@testable import BinaryCoder

import Foundation

class rosswiftTests: XCTestCase {


    func testResp() {
        let response = """
    <?xml version='1.0'?>
    <methodResponse>
    <params>
    <param>
    <value><array><data>
    <value><int>1</int></value>
    <value><string>Registered [/talker] as publisher of [/chatter]</string></value>
    <value><array><data></data></array></value></data></array></value>
    </param>
    </params>
    </methodResponse>
    """


        if let a = XMLRPCClient.parseResponse(xml: response) {
            XCTAssertEqual(a.size(), 3)
            XCTAssertEqual(a[0].int!, 1)
            XCTAssertEqual(a[1].string, "Registered [/talker] as publisher of [/chatter]")
        } else {
            XCTFail()
        }
    }

    func testResponse() {
        let response = """
                <?xml version='1.0'?>
                <methodResponse>
                <params>
                <param>
                <value><array><data>
                <value><int>1</int></value>
                <value><string>Registered [/talker] as provider of [/talker/debug/close_all_connections]</string></value>
                <value><int>1</int></value>
                </data></array></value>
                </param>
                </params>
                </methodResponse>
                """

        if let a = XMLRPCClient.parseResponse(xml: response) {
            XCTAssertEqual(a.size(), 3)
            XCTAssertEqual(a[0].int!, 1)
            XCTAssertEqual(a[1].string, "Registered [/talker] as provider of [/talker/debug/close_all_connections]")
            XCTAssertEqual(a[2].int!, 1)
        } else {
            XCTFail()
        }
    }

    func testXmlStruct() {
        let r = "<value><struct><member><name>adam</name><value><dateTime.iso8601>19831212T14:13:12</dateTime.iso8601></value></member><member><name>bertil</name><value><i4>2345</i4></value></member></struct></value>"
        var seq = r.dropFirst(0)
        var v = XmlRpcValue()
        let res = v.fromXML(xml: &seq)
        XCTAssert(res)
        if let s = v.dictionary {
            XCTAssertEqual(s["bertil"]!.int!,2345)
            XCTAssertEqual(s["adam"]!.date!.description,"1983-12-12 14:13:12 +0000")
        } else {
            XCTFail()
        }
    }

    func testResp1() {
        let r = "<?xml version=\'1.0\'?>\n<methodResponse>\n<params>\n<param>\n<value><array><data>\n<value><int>1</int></value>\n<value><string>/tcp_keepalive</string></value>\n<value><boolean>0</boolean></value>\n</data></array></value>\n</param>\n</params>\n</methodResponse>\n"
        if let response = XMLRPCClient.parseResponse(xml: r) {
            XCTAssertEqual(response.size(), 3)
            XCTAssertEqual(response[0].int!, 1)
            XCTAssertEqual(response[1].string, "/tcp_keepalive")
            XCTAssertEqual(response[2].bool!, false)
        } else {
            XCTFail()
        }
    }


    func testSerialization() {

        let response = "<methodResponse><params><param><value><array><data>"
            + "<value><i4>1</i4></value><value><string></string></value><value>"
            + "<array><data><value><string>TCPROS</string></value><value>"
            + "<string>B2036.local</string></value><value><i4>54794</i4>"
            + "</value></data></array></value></data></array></value>"
            + "</param></params></methodResponse>"

        if let response = XMLRPCClient.parseResponse(xml: response) {
            XCTAssertEqual(response.size(), 3)
            XCTAssertEqual(response[0].int!, 1)
            XCTAssertEqual(response[1].string, "")
            XCTAssertEqual(response[2].size(), 3)
            XCTAssertEqual(response[2][0].string, "TCPROS")
            XCTAssertEqual(response[2][1].string, "B2036.local")
            XCTAssertEqual(response[2][2].int!, 54794)
        } else {
            XCTFail()
        }



    }

    func testExample() {
        // This is an example of a functional test case.
        // Use XCTAssert and related functions to verify your tests produce the correct
        // results.

        let xml = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><methodCall>"
            + "<methodName>requestTopic</methodName><params><param>"
            + "<value>/matlab_global_node_02713</value></param><param>"
            + "<value>/chatter</value></param><param><value><array><data>"
            + "<value><array><data><value>TCPROS</value></data></array>"
            + "</value></data></array></value></param></params></methodCall>"

            let ob = XMLRPCManager.parseRequest(xml: xml)
            XCTAssertEqual(ob.method, "requestTopic")
            XCTAssertEqual(ob.params[0].string, "/matlab_global_node_02713")
            XCTAssertEqual(ob.params[1].string, "/chatter")
            XCTAssertEqual(ob.params[2][0].string, "TCPROS")



    }

    func testSerMess() {
        let mes = std_msgs.string("long string with text")
        let s = SerializedMessage(msg: mes)
        XCTAssertEqual(s.buf,[25, 0, 0, 0, 21, 0, 0, 0,
                              108, 111, 110, 103, 32, 115, 116, 114, 105, 110,
                              103, 32, 119, 105, 116, 104, 32, 116, 101, 120, 116])

        let data = try! BinaryEncoder.encode(mes)
        var count = try! BinaryEncoder.encode(UInt32(data.count))
        count.append(contentsOf: data)
        XCTAssertEqual(count, s.buf)
    }

    func testXmlValue() {
        let v = XmlRpcValue(any: 2.34)
        var d = 1.23
        XCTAssert( v.get(val: &d) )
        XCTAssertEqual(2.34, d)

        let vb = XmlRpcValue(any: true)
        var db = false
        XCTAssert( vb.get(val: &db) )
        XCTAssertEqual(true, db)

        let vv = [12,23,4,647]
        let vec = XmlRpcValue(any: vv)
        var ivec = [Int]()
        XCTAssert( vec.get(val: &ivec) )
        XCTAssertEqual(vv, ivec)

        let vc : [Double] = [12,23,4,647]
        var dvec = [Double]()
        XCTAssert( vec.get(val: &dvec) )
        XCTAssertEqual(vc, dvec)

        let vc2 = [12.34,0.23,234,0.647]
        let vec2 = XmlRpcValue(any: vc2)
        var d2 = [Double]()
        XCTAssert( vec2.get(val: &d2) )
        XCTAssertEqual(vc2, d2)

    }


    static var allTests = [
        ("testResp",testResp),
        ("testResponse",testResponse),
        ("testSerialization",testSerialization),
        ("testExample", testExample),
        ("testXmlValue",testXmlValue),
        ("testSerMess",testSerMess)
    ]
}
