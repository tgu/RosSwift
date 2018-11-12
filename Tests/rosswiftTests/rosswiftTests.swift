import XCTest
@testable import RosSwift
@testable import XMLRPCSerialization
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


        if let d = response.data(using: .utf8) {
            do {
                let r = try XMLRPCSerialization.xmlrpcResponse(from: d )
                if case .response(let a) = r {
                    XCTAssertEqual(a.count, 1)
                    let p = a[0] as! [Any]
                    XCTAssertEqual(p.count, 3)
                    XCTAssertEqual(p[0] as! UInt, 1)
                    XCTAssertEqual(p[1] as! String, "Registered [/talker] as publisher of [/chatter]")
                    let c = p[2] as! [Any]
                    XCTAssertEqual(c.count, 0)
                } else {
                    XCTFail()
                }
            } catch {
                XCTFail(error.localizedDescription)
            }
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

        if let d = response.data(using: .utf8) {
            do {
                let r = try XMLRPCSerialization.xmlrpcResponse(from: d )
                if case .response(let a) = r {
                    XCTAssertEqual(a.count, 1)
                    let p = a[0] as! [Any]
                    XCTAssertEqual(p.count, 3)
                    XCTAssertEqual(p[0] as! UInt, 1)
                    XCTAssertEqual(p[1] as! String, "Registered [/talker] as provider of [/talker/debug/close_all_connections]")
                    XCTAssertEqual(p[2] as! UInt, 1)
                } else {
                    XCTFail()
                }
            } catch {
                XCTFail(error.localizedDescription)
            }
        }

    }


    func testSerialization() {

        let response = "<methodResponse><params><param><value><array><data>"
            + "<value><i4>1</i4></value><value><string></string></value><value>"
            + "<array><data><value><string>TCPROS</string></value><value>"
            + "<string>B2036.local</string></value><value><i4>54794</i4>"
            + "</value></data></array></value></data></array></value>"
            + "</param></params></methodResponse>"

        if let d = response.data(using: .utf8) {
            do {
                let r = try XMLRPCSerialization.xmlrpcResponse(from: d )
            } catch {
                XCTFail(error.localizedDescription)
            }
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

        let xmlproto = "<value><array><data><value><array><data><value>"
            + "<string>TCPROS</string></value></data></array></value>"
            + "</data></array></value>"

        let xml2 = """
        <methodResponse><params><param>
            <value><array><data><value><i4>1</i4></value><value></value>
            <value><array><data><value>TCPROS</value><value>B2036.local</value>
            <value><i4>60331</i4></value></data></array>
            </value></data></array></value>
        </param></params></methodResponse>
        """

        let xml3 = "<methodResponse><params><param><value><array><data>"
            + "<value><i4>1</i4></value><value>Success</value><value>"
            + "<array><data/></array></value></data></array></value>"
            + "</param></params></methodResponse>"

        guard let data3 = xml3.data(using: .utf8) else {
            XCTFail("xml3 failed")
            return
        }

        let xp3 = XMLParser(data: data3)
        do {
            let obj = try XMLRPCSerialization.xmlrpcResponse(from: data3)
            //            XCTAssertEqual(obj, "requestTopic")
            //            XCTAssertEqual(obj.params[0] as! String, "/matlab_global_node_02713")
            //            XCTAssertEqual(obj.params[1] as! String, "/chatter")
            //            XCTAssertEqual(obj.params[2] as! [[String]], [["TCPROS"]])

        }

        catch {
            print(error)
            XCTFail("xp3 failed")
        }

        guard let data2 = xml2.data(using: .utf8) else {
            XCTFail("xml2 failed")
            return
        }


        guard let data = xml.data(using: .utf8) else {
            XCTFail("xml failed")
            return
        }

        let xp = XMLParser(data: data)

        do {

            let obj = try XMLRPCSerialization.xmlrpcRequest(from: data)
            XCTAssertEqual(obj.methodName, "requestTopic")
            XCTAssertEqual(obj.params[0] as! String, "/matlab_global_node_02713")
            XCTAssertEqual(obj.params[1] as! String, "/chatter")
            XCTAssertEqual(obj.params[2] as! [[String]], [["TCPROS"]])

            let val = XmlRpcValue(anyArray: obj.params)

            let p = obj.params[2]
            let proto = XmlRpcValue(any: p)
            let p1 = proto[0]
            if case .array = p1.value {} else {
                XCTFail("requestTopic protocol list was not a list of lists")
            }

            if case .string = p1[0].value {} else {
                XCTFail( "requestTopic received a protocol list in which a sublist did not start with a string")
            }


            XCTAssertEqual(val[0].string, "/matlab_global_node_02713")
            XCTAssertEqual(val[1].string, "/chatter")
            XCTAssertEqual(val.size(), 3)

            let x = proto.toXml()
            XCTAssertEqual(xmlproto, x)

            let obj2 = try XMLRPCSerialization.xmlrpcResponse(from: data2)


        } catch {
            XCTFail()
        }


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
