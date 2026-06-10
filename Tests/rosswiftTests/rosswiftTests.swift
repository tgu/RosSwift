import Testing
@testable import RosSwift
@testable import StdMsgs
@testable import BinaryCoder
import rpcobject
import Logging

import Foundation

@Suite("RosSwift tests")
struct rosswiftTests {

    @Test func resp() throws {
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

        let a = try #require(XMLRPCClient.parseResponse(xml: response))
        #expect(a.size() == 3)
        #expect(a[0].int! == 1)
        #expect(a[1].string == "Registered [/talker] as publisher of [/chatter]")
    }

    @Test func response() throws {
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

        let a = try #require(XMLRPCClient.parseResponse(xml: response))
        #expect(a.size() == 3)
        #expect(a[0].int! == 1)
        #expect(a[1].string == "Registered [/talker] as provider of [/talker/debug/close_all_connections]")
        #expect(a[2].int! == 1)
    }

    @Test func xmlStruct() throws {
        let r = "<value><struct><member><name>adam</name><value><dateTime.iso8601>19831212T14:13:12</dateTime.iso8601></value></member><member><name>bertil</name><value><i4>2345</i4></value></member></struct></value>"
        var seq = r.dropFirst(0)
        var v = XmlRpcValue()
        let res = v.fromXML(xml: &seq)
        #expect(res)
        let s = try #require(v.dictionary)
        #expect(s["bertil"]!.int! == 2345)
        #expect(s["adam"]!.date!.description == "1983-12-12 14:13:12 +0000")
    }

    @Test func resp1() throws {
        let r = "<?xml version=\'1.0\'?>\n<methodResponse>\n<params>\n<param>\n<value><array><data>\n<value><int>1</int></value>\n<value><string>/tcp_keepalive</string></value>\n<value><boolean>0</boolean></value>\n</data></array></value>\n</param>\n</params>\n</methodResponse>\n"
        let response = try #require(XMLRPCClient.parseResponse(xml: r))
        #expect(response.size() == 3)
        #expect(response[0].int! == 1)
        #expect(response[1].string == "/tcp_keepalive")
        #expect(response[2].bool! == false)
    }

    @Test func serialization() throws {
        let response = "<methodResponse><params><param><value><array><data>"
            + "<value><i4>1</i4></value><value><string></string></value><value>"
            + "<array><data><value><string>TCPROS</string></value><value>"
            + "<string>B2036.local</string></value><value><i4>54794</i4>"
            + "</value></data></array></value></data></array></value>"
            + "</param></params></methodResponse>"

        let parsed = try #require(XMLRPCClient.parseResponse(xml: response))
        #expect(parsed.size() == 3)
        #expect(parsed[0].int! == 1)
        #expect(parsed[1].string == "")
        #expect(parsed[2].size() == 3)
        #expect(parsed[2][0].string == "TCPROS")
        #expect(parsed[2][1].string == "B2036.local")
        #expect(parsed[2][2].int! == 54794)
    }

    @Test func example() {
        let xml = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><methodCall>"
            + "<methodName>requestTopic</methodName><params><param>"
            + "<value>/matlab_global_node_02713</value></param><param>"
            + "<value>/chatter</value></param><param><value><array><data>"
            + "<value><array><data><value>TCPROS</value></data></array>"
            + "</value></data></array></value></param></params></methodCall>"

        let ob = XmlRpcUtil.parseRequest(xml: xml)
        #expect(ob.method == "requestTopic")
        #expect(ob.params[0].string == "/matlab_global_node_02713")
        #expect(ob.params[1].string == "/chatter")
        #expect(ob.params[2][0].string == "TCPROS")
    }

    @Test func serMess() throws {
        let mes = std_msgs.string("long string with text")
        let s = SerializedMessage(msg: mes)
        #expect(s.buf == [25, 0, 0, 0, 21, 0, 0, 0,
                          108, 111, 110, 103, 32, 115, 116, 114, 105, 110,
                          103, 32, 119, 105, 116, 104, 32, 116, 101, 120, 116])

        let data = try BinaryEncoder.encode(mes)
        var count = try BinaryEncoder.encode(UInt32(data.count))
        count.append(contentsOf: data)
        #expect(count == s.buf)
    }

    @Test func xmlValue() {
        let v = XmlRpcValue(any: 2.34)
        var d = 1.23
        #expect(v.get(val: &d))
        #expect(2.34 == d)

        let vb = XmlRpcValue(any: true)
        var db = false
        #expect(vb.get(val: &db))
        #expect(true == db)

        let vv = [12,23,4,647]
        let vec = XmlRpcValue(any: vv)
        var ivec = [Int]()
        #expect(vec.get(val: &ivec))
        #expect(vv == ivec)

        let vc : [Double] = [12,23,4,647]
        var dvec = [Double]()
        #expect(vec.get(val: &dvec))
        #expect(vc == dvec)

        let vc2 = [12.34,0.23,234,0.647]
        let vec2 = XmlRpcValue(any: vc2)
        var d2 = [Double]()
        #expect(vec2.get(val: &d2))
        #expect(vc2 == d2)
    }

    @Test func setLoggerLevel() {
        let original = logger.logLevel
        defer { Console.setLoggerLevel(logger: "ros", level: original) }

        Console.setLoggerLevel(logger: "ros", level: .warning)
        #expect(logger.logLevel == .warning)

        Console.setLoggerLevel(logger: "ros", level: .debug)
        #expect(logger.logLevel == .debug)
    }
}
