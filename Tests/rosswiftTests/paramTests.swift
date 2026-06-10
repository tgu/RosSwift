//
//  paramTests.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-10-30.
//

import Testing
@testable import RosSwift
@testable import RosTime
@testable import rpcobject
import RosNetwork
import rosmaster


@Suite("Parameter tests", .serialized)
class paramTests {
    var ros: Ros!
    let master: rosmaster.Master
    var remap: [String:String] {
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
        master = rosmaster.Master(host: network.gHost, port: 0, advertise: false)
        _ = try await master.start().get()
        // Put setup code here. This method is called before the invocation of each test method in the class.

        ros = try! Ros(name: "paramTests", master: host, port: port)

        await ros.param.set(key: "string", value: "test")
        await ros.param.set(key: "int", value: Int(10))
        await ros.param.set(key: "double", value: Double(10.5))
        await ros.param.set(key: "bool", value: false)
        _ = await ros.param.del(key: "/test_set_param_setThenGetStringCached")
        _ = await ros.param.del(key: "/test_create_parameter")

    }

    deinit {
        ros.shutdown()
        _ = master.stop()
    }
    
    @Test func testAllParamTypes() async {
        var string_param = ""
        #expect( await ros.param.get( "string", &string_param ) )
        #expect( string_param == "test" )

        var int_param = 0
        #expect( await ros.param.get( "int", &int_param ) )
        #expect( int_param == 10 )

        var double_param = 0.0
        #expect( await ros.param.get( "double", &double_param ) )
        #expect( double_param == 10.5 )

        var bool_param = true
        #expect( await ros.param.get( "bool", &bool_param ) )
        #expect( bool_param  == false)
    }

    @Test func testSetThenGetString() async {
        await ros.param.set( key: "test_set_param", value: "asdf" )

        var param = ""
        #expect( await ros.param.get( "test_set_param", &param ) )
        #expect( "asdf" == param )

        var v = XmlRpcValue()
        #expect( await ros.param.get("test_set_param", &v) )
        #expect(v == XmlRpcValue(str: "asdf") )
    }

    @Test func testSetThenGetStringCached() async {
        _ = await ros.param.del(key: "test_set_param_setThenGetStringCached")
        var  param = ""
        #expect( await ros.param.getCached( "test_set_param_setThenGetStringCached", &param) == false )

        await ros.param.set( key: "test_set_param_setThenGetStringCached", value: "asdf" )
        #expect( await ros.param.getCached( "test_set_param_setThenGetStringCached", &param) )
        #expect( "asdf" == param )
    }

    @Test func testSetThenGetNamespaceCached() async
    {
        _ = await ros.param.del(key: "/test_set_param_setThenGetStringCached2")
        var stringParam = ""
        var structParam = XmlRpcValue()
        let ns = "test_set_param_setThenGetStringCached2"
        #expect(await ros.param.getCached(ns, &stringParam) == false)

        await ros.param.set(key: ns, value: "a")
        #expect(await ros.param.getCached(ns, &stringParam))
        #expect("a" == stringParam)

        await ros.param.set(key: ns + "/foo", value: "b")
        #expect(await ros.param.getCached(ns + "/foo", &stringParam))
        #expect("b" == stringParam)
        #expect(await ros.param.getCached(ns, &structParam))
        #expect(structParam.hasMember("foo"))
        #expect("b" == structParam["foo"]?.string)

    }

    @Test func testSetThenGetCString() async
    {
        await ros.param.set( key: "test_set_param", value: "asdf" )
        var param = ""
        #expect( await ros.param.get( "test_set_param", &param ) )
        #expect( "asdf" == param )
    }

    @Test func testsetThenGetInt() async
    {
        await ros.param.set( key: "test_set_param", value: 42)
        var param = 0
        #expect( await ros.param.get( "test_set_param", &param ) )
        #expect( 42 == param )
        var v = XmlRpcValue()
        #expect(await ros.param.get("test_set_param", &v))
        guard case .int = v else {
            #expect(Bool(false))
            return
        }
    }

    @Test func testunknownParam() async
    {
        var param = ""
        #expect( await ros.param.get( "this_param_really_should_not_exist", &param ) == false )
    }

    @Test func testdeleteParam() async
    {
        await ros.param.set( key: "test_delete_param", value: "asdf" )
        _ = await ros.param.del( key: "test_delete_param" )
        var param = ""
        #expect( await ros.param.get( "test_delete_param", &param ) == false )
    }

    @Test func testhasParam() async
    {
        #expect( await ros.param.has( key: "string" ) )
    }

    @Test func testsetIntDoubleGetInt() async
    {
        await ros.param.set(key: "test_set_int_as_double", value: 1)
        await ros.param.set(key: "test_set_int_as_double", value: 3.0)

        var i = -1
        #expect(await ros.param.get("test_set_int_as_double", &i))
        #expect(3 == i)
        var d = 0.0
        #expect(await ros.param.get("test_set_int_as_double", &d))
        #expect(3.0 == d)
    }

    @Test func testgetIntAsDouble() async
    {
        await ros.param.set(key: "int_param", value: 1)
        var d = 0.0
        #expect(await ros.param.get("int_param", &d))
        #expect(1.0 == d)
    }

    @Test func testgetDoubleAsInt() async
    {
        await ros.param.set(key: "double_param", value: 2.3)
        var i = -1
        #expect(await ros.param.get("double_param", &i))
        #expect(2 == i)

        await ros.param.set(key: "double_param", value: 3.8)
        i = -1
        #expect(await ros.param.get("double_param", &i))
        #expect(4 == i)
    }

    @Test func testsearchParam() async
    {
        let ns = "/a/b/c/d/e/f"
        var result = ""

        await ros.param.set(key: "/s_i", value: 1)
        #expect(await ros.param.search(ns: ns, key: "s_i", result: &result))
        #expect(result == "/s_i")
        #expect(await ros.param.del(key: "/s_i"))

        await ros.param.set(key: "/a/b/s_i", value: 1)
        #expect(await ros.param.search(ns: ns, key: "s_i", result: &result))
        #expect(result == "/a/b/s_i")
        #expect(await ros.param.del(key: "/a/b/s_i"))

        await ros.param.set(key: "/a/b/c/d/e/f/s_i", value: 1)
        #expect(await ros.param.search(ns: ns, key: "s_i", result: &result))
        #expect(result == "/a/b/c/d/e/f/s_i")
        #expect(await ros.param.del(key: "/a/b/c/d/e/f/s_i"))

        #expect(await ros.param.search(ns: ns, key: "s_j", result: &result) == false)
    }

    @Test func testsearchParamNodeHandle() async
    {
        guard let n = await ros.createNode(ns: "/a/b/c/d/e/f") else {
            #expect(Bool(false))
            return
        }
        var result = ""

        await n.set(parameter: "/s_i", value: 1)
        let ok = await n.search(parameter: "s_i", result: &result)
        #expect(ok)
        #expect(result == "/s_i")
        #expect(await n.delete(paramter: "/s_i"))

        await n.set(parameter: "/a/b/s_i", value: 1)
        await #expect(n.search(parameter: "s_i", result: &result))
        #expect(result == "/a/b/s_i")
        #expect(await n.delete(paramter:"/a/b/s_i"))

        await n.set(parameter: "/a/b/c/d/e/f/s_i", value: 1)
        await #expect(n.search(parameter: "s_i", result: &result))
        #expect(result == "/a/b/c/d/e/f/s_i")
        #expect(await n.delete(paramter:"/a/b/c/d/e/f/s_i"))

        await #expect(n.search(parameter: "s_j", result: &result) == false)
    }

    @Test func testsearchParamNodeHandleWithRemapping() async
    {
        _ = await ros.param.del(key: "/s_b")
        let remappings = ["s_c":"s_b"]
        guard let n = await ros.createNode(ns: "/a/b/c/d/e/f", remappings: remappings) else {
            #expect(Bool(false))
            return
        }
        var result = ""

        await n.set(parameter: "/s_c", value: 1)
        await #expect(n.search(parameter: "s_c", result: &result) == false)
        await n.set(parameter: "/s_b", value: 1)
        await #expect(n.search(parameter: "s_c", result: &result))
        print("RESULT \(result)")
    }

    @Test func testgetMissingXmlRpcValueParameterCachedTwice() async
    {
        var v = XmlRpcValue()
        #expect(await ros.param.getCached("invalid_xmlrpcvalue_param", &v) == false)
        #expect(await ros.param.getCached("invalid_xmlrpcvalue_param", &v) == false)
    }

    @Test func testdoublePrecision() async
    {
        await ros.param.set(key: "bar", value: 0.123456789123456789)
        var d = 0.0
        #expect(await ros.param.get("bar", &d))
        #expect(d == 0.12345678912345678)
    }

    var vec_s = [String]()
    var vec_s2 = [String]()
    var vec_d = [Double]()
    var vec_d2 = [Double]()
    var vec_f = [Float32]()
    var vec_f2 = [Float32]()
    var vec_i = [Int]()
    var vec_i2 = [Int]()
    var vec_b = [Bool]()
    var vec_b2 = [Bool]()

    @Test func testvectorStringParam() async
    {
        let param_name = "v_param"


        vec_s = ["foo","bar","baz"]

        await ros.param.set(key: param_name, value: vec_s)

        #expect(await ros.param.get(param_name, &vec_d) == false)
        #expect(await ros.param.get(param_name, &vec_f) == false)
        #expect(await ros.param.get(param_name, &vec_i) == false)
        #expect(await ros.param.get(param_name, &vec_b) == false)

        #expect(await ros.param.get(param_name, &vec_s2))

        #expect(vec_s.count == vec_s2.count)
        #expect(vec_s == vec_s2)

        // test empty vector
        vec_s.removeAll()
        await ros.param.set(key: param_name, value: vec_s)
        #expect(await ros.param.get(param_name, &vec_s2))
        #expect(vec_s.count == vec_s2.count)
    }

    @Test func testvectorDoubleParam() async
    {
        let param_name = "vec_double_param"

        vec_d = [-0.123456789,3,3.01,7.01]

        await ros.param.set(key: param_name, value: vec_d)

        #expect(await ros.param.get(param_name, &vec_s) == false)
        #expect(await ros.param.get(param_name, &vec_i))
        #expect(await ros.param.get(param_name, &vec_b))
        #expect(await ros.param.get(param_name, &vec_f))

        #expect(await ros.param.get(param_name, &vec_d2))

        #expect(vec_d.count == vec_d2.count)
        #expect(vec_d == vec_d2)
        #expect(vec_f == [-0.123456789,3,3.01,7.01])
        #expect(vec_i == [0,3,3,7])
        #expect(vec_b == [true,true,true,true])

    }

    @Test func testvectorFloatParam() async
    {
        let param_name = "vec_float_param"

        vec_f = [-0.25, 0.0, 3, 3.25]

        await ros.param.set(key: param_name, value: vec_f)

        #expect(await ros.param.get(param_name, &vec_s) == false)
        #expect(await ros.param.get(param_name, &vec_i))
        #expect(await ros.param.get(param_name, &vec_b))
        #expect(await ros.param.get(param_name, &vec_d))

        #expect(vec_b == [true,false,true,true])
        #expect(vec_i == [0,0,3,3])
        #expect(vec_d == [-0.25, 0.0, 3, 3.25])

        #expect(await ros.param.get(param_name, &vec_f2))

        #expect(vec_f.count == vec_f2.count)
        #expect(vec_f == vec_f2)
    }

    @Test func testvectorIntParam() async
    {
        let param_name = "vec_int_param"

        vec_i = [-1, 0, 1337, 2]

        await ros.param.set(key: param_name, value: vec_i)

        #expect(await ros.param.get(param_name, &vec_s) == false)
        #expect(await ros.param.get(param_name, &vec_d))
        #expect(await ros.param.get(param_name, &vec_f))
        #expect(await ros.param.get(param_name, &vec_b))

        #expect(vec_b == [true,false,true,true])
        #expect(vec_f == [-1,0,1337,2])
        #expect(vec_d == [-1,0,1337,2])

        #expect(await ros.param.get(param_name, &vec_i2))

        #expect(vec_i.count == vec_i2.count)
        #expect(vec_i == vec_i2)
    }

    @Test func testvectorBoolParam() async
    {
        let param_name = "vec_bool_param"

        vec_b = [true, false, true, true]

        await ros.param.set(key: param_name, value: vec_b)

        #expect(await ros.param.get(param_name, &vec_s) == false)
        #expect(await ros.param.get(param_name, &vec_d))
        #expect(await ros.param.get(param_name, &vec_f))
        #expect(await ros.param.get(param_name, &vec_i))

        #expect(vec_i == [1,0,1,1])
        #expect(vec_d == [1,0,1,1])
        #expect(vec_f == [1,0,1,1])


        #expect(await ros.param.get(param_name, &vec_b2))

        #expect(vec_b.count == vec_b2.count)
        #expect(vec_b == vec_b2)
    }

    var map_s = [String:String]()
    var map_s2 = [String:String]()
    var map_d = [String:Double]()
    var map_d2 = [String:Double]()
    var map_f = [String:Float32]()
    var map_f2 = [String:Float32]()
    var map_i = [String:Int]()
    var map_i2 = [String:Int]()
    var map_b = [String:Bool]()
    var map_b2 = [String:Bool]()

    @Test func testmapStringParam() async
    {
        let param_name = "map_str_param"

        map_s = ["a": "apple", "b": "blueberry", "c": "carrot"]

        await ros.param.set(key: param_name, value: map_s)

        #expect(await ros.param.get(param_name, &map_d) == false)
        #expect(await ros.param.get(param_name, &map_f) == false)
        #expect(await ros.param.get(param_name, &map_i) == false)
        #expect(await ros.param.get(param_name, &map_b) == false)

        #expect(await ros.param.get(param_name, &map_s2))

        #expect(map_s.count == map_s2.count)
        #expect(map_s == map_s2)
    }

    @Test func testmapDoubleParam() async
    {
        let param_name = "map_double_param"

        map_d = ["a":0.0,"b":-0.123456789,"c":12345678]

        await ros.param.set(key: param_name, value: map_d)

        #expect(await ros.param.get(param_name, &map_s) == false)
        #expect(await ros.param.get(param_name, &map_f))
        #expect(await ros.param.get(param_name, &map_i))
        #expect(await ros.param.get(param_name, &map_b))
        #expect(await ros.param.get(param_name, &map_d2))

        #expect(map_f == ["a":0.0,"b":-0.123456789,"c":12345678])
        #expect(map_i == ["a":0,"b":0,"c":12345678])
        #expect(map_b == ["a":false,"b":true,"c":true])

        #expect(map_d.count == map_d2.count)
        #expect(map_d == map_d2)
    }

    @Test func testmapFloatParam() async
    {
        let param_name = "map_float_param"

        map_f = ["a": 0.0, "b":-0.25,"c":1234567]

        await ros.param.set(key: param_name, value: map_f)

        #expect(await ros.param.get(param_name, &map_s) == false)
        #expect(await ros.param.get(param_name, &map_d))
        #expect(await ros.param.get(param_name, &map_i))
        #expect(await ros.param.get(param_name, &map_b))

        #expect(map_d == ["a":0.0,"b":-0.25,"c":1234567])
        #expect(map_i == ["a":0,"b":0,"c":1234567])
        #expect(map_b == ["a":false,"b":true,"c":true])

        #expect(await ros.param.get(param_name, &map_f2))

        #expect(map_f.count == map_f2.count)
        #expect(map_f == map_f2)
    }

    @Test func testmapIntParam() async
    {
        let param_name = "map_int_param"

        map_i = ["a":0, "b":-1, "c":1337]

        await ros.param.set(key: param_name, value: map_i)

        #expect(await ros.param.get(param_name, &map_s) == false)
        #expect(await ros.param.get(param_name, &map_d))
        #expect(await ros.param.get(param_name, &map_f))
        #expect(await ros.param.get(param_name, &map_b))

        #expect(map_f == ["a":0.0,"b":-1,"c":1337])
        #expect(map_d == ["a":0,"b":-1,"c":1337])
        #expect(map_b == ["a":false,"b":true,"c":true])

        #expect(await ros.param.get(param_name, &map_i2))

        #expect(map_i.count == map_i2.count)
        #expect(map_i == map_i2)
    }

    @Test func testmapBoolParam() async
    {
        let param_name = "map_bool_param"

        map_b = ["a":true, "b":false, "c":true]

        await ros.param.set(key: param_name, value: map_b)

        #expect(await ros.param.get(param_name, &map_s) == false)
        #expect(await ros.param.get(param_name, &map_d))
        #expect(await ros.param.get(param_name, &map_f))
        #expect(await ros.param.get(param_name, &map_i))

        #expect(map_i == ["a":1,"b":0, "c":1])
        #expect(map_f == ["a":1,"b":0, "c":1])
        #expect(map_d == ["a":1,"b":0, "c":1])

        #expect(await ros.param.get(param_name, &map_b2))

        #expect(map_b.count ==  map_b2.count)
        #expect(map_b ==  map_b2)
    }

    @Test func testparamTemplateFunction() async
    {
        #expect( await ros.param.param( name: "string", defaultValue: "" ) == "test" )
        #expect( await ros.param.param( name: "gnirts", defaultValue: "test" ) == "test" )

        #expect( await ros.param.param( name: "int", defaultValue: 0 ) == 10 )
        #expect( await ros.param.param( name: "tni", defaultValue: 10 ) == 10 )

        #expect( await ros.param.param( name: "double", defaultValue: 0.0 ) == 10.5 )
        #expect( await ros.param.param( name: "elbuod", defaultValue: 10.5 ) == 10.5 )

        #expect( await ros.param.param( name: "bool", defaultValue: true ) == false )
        #expect( await ros.param.param( name: "loob", defaultValue: true ) == true )
    }

    @Test func testparamNodeHandleTemplateFunction() async
    {
        let nh = await ros.createNode()

        #expect( await nh.param( name: "string", defaultValue: "" ) == "test" )
        #expect( await nh.param( name: "gnirts", defaultValue: "test" ) == "test" )

        #expect( await nh.param( name: "int", defaultValue: 0 ) == 10 )
        #expect( await nh.param( name: "tni", defaultValue: 10 ) == 10 )

        #expect( await nh.param( name: "double", defaultValue: 0.0 ) == 10.5 )
        #expect( await nh.param( name: "elbuod", defaultValue: 10.5 ) == 10.5 )

        #expect( await nh.param( name: "bool", defaultValue: true ) == false )
        #expect( await nh.param( name: "loob", defaultValue: true ) == true )
    }

    // should run last, tests runs in alphabetical order (unless random order is selected)
    @Test func testZgetParamNames() async {
        var test_params = [String]()
        let b = await ros.param.getParamNames(keys: &test_params)


        #expect(b)
        #expect(10 > test_params.count)

        let h = try! await ros.param.getParameterNames()
        #expect(10 > h.count)
        print(h)
}


}
