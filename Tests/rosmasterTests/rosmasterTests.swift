import XCTest

@testable import rosmaster
@testable import rpcobject


extension Int: ArrayConstructable, Collection {
    public subscript(position: Int) -> Int {
        get {
            return self
        }
        set {
            
        }
    }
    
    public typealias Index = Int

    public init(any: Any) {
        self = 0
    }

    public var count: Int {
        return 1
    }
    
    public func index(after i: Int) -> Int {
        return i + 1
    }
    
    public var startIndex: Int {
        return 0
    }
    
    public var endIndex: Int {
        return 0
    }

    public var dictionary: [String : Int]? {
        return nil
    }

}

class RosmasterTests: XCTestCase {

    func testLongName() {
        let r = RadixTree<Int>()
        XCTAssertNotNil(r.insert("top/name/space/with/variable/ens", value: 456).self)
        XCTAssertEqual(r.height, 6)
        let g1 = r.find("top/name/space/with/variable")
        let g2 = r.find("top/name/space/with")
        let g3 = r.find("top/name/space")
        let g4 = r.find("top/name")
        let g5 = r.find("top")
        let g6 = r.find("")
        let g7 = r.find("top/name/spacer")

        XCTAssertTrue(g1)
        XCTAssertTrue(g2)
        XCTAssertTrue(g3)
        XCTAssertTrue(g4)
        XCTAssertTrue(g5)
        XCTAssertTrue(g6)
        XCTAssertFalse(g7)
    }

    func testGetLongName() {
        let r = RadixTree<Int>()
        XCTAssertNotNil(r.insert("top/name/space/with/variable/ens", value: 456).self)
        XCTAssertNotNil(r.insert("top/name/space/with/var", value: 999).self)
        XCTAssertNotNil(r.insert("top/name/here", value: 123).self)
        XCTAssertEqual(r.height, 6)
        let g1 = r.get("top/name/space/with/variable")
        let g2 = r.get("top/name/space/with")
        let g3 = r.get("top/name/space")
        let g4 = r.get("top/name")
        let g5 = r.get("top")
        let g6 = r.get("")
        let g7 = r.get("top/name/spacer")


        XCTAssertNotNil(g1)
        XCTAssertNotNil(g2)
        XCTAssertNotNil(g3)
        XCTAssertNotNil(g4)
        XCTAssertNotNil(g5)
        XCTAssertNotNil(g6)
        XCTAssertNil(g7)

        XCTAssertEqual(g1?.children.count, 1)
        XCTAssertEqual(g2?.children.count, 2)
        XCTAssertEqual(g4?.children.count, 2)
    }



    func testParameters()  {
        let r = RadixTree<Int>()

        XCTAssertNotNil(r.insert("rom/nus", value: 1).self)

        if let g = r.get("rom/nus") as? Edge<Int> {
            print("rom/nus has value \(g.value)")
        }

        if let g = r.get("rom") {
            print("rom has value \(g.children.count) children")
        }

        r.insert("rub/icundus", value: 2)
        r.insert("rub/icon", value: 3)
        r.insert("rom/ane", value: 4)
        r.insert("rub/er", value: 5)
        r.insert("rub/ens", value: 6)
        r.insert("top/name/space/with/variable/ens", value: 456)
        r.printTree()
        r.insert("top/name/space/nr", value: 999)
        r.printTree()
        print(r.getNames())

        print("\n\nFIND TESTS")
        XCTAssertFalse(r.find("courts")) // false
        XCTAssertFalse(r.find("r")) // true
        XCTAssertFalse(r.find("ro")) // true
        XCTAssertTrue(r.find("rom")) // true
        XCTAssertTrue(r.find("rom")) // true
        XCTAssertTrue(r.find("top/name/space/with/variable/ens")) // true
        XCTAssertFalse(r.find("roman")) // true
        XCTAssertTrue(r.find("rom/ane")) // true
        XCTAssertFalse(r.find("romans")) // false
        XCTAssertFalse(r.find("steve")) // true
        print("\n\nREMOVE TESTS")

        XCTAssertNil(r.remove("c"))
        XCTAssertNotNil(r.remove("rub"))
        XCTAssertTrue(r.find("rom/ane")) // true
        r.printTree()
        XCTAssertNil(r.remove("stevenson"))
        XCTAssertTrue(r.find("rom")) // true
        XCTAssertNotNil(r.remove("rom"))
        XCTAssertFalse(r.find("rom/ane")) // true
        r.printTree()

    }

    private func notify(updates: [Update<Int>]) {
        updates.forEach { print($0) }
    }

    func testResolveNames() {
        let node1 = "/node1"
        let node2 = "/wg/node2"
        let node3 = "/wg/node3"

        // relative names

        XCTAssertEqual(resolve(name: "bar", nameSpace: node1), "/bar")
        XCTAssertEqual(resolve(name: "bar", nameSpace: node2), "/wg/bar")
        XCTAssertEqual(resolve(name: "foo/bar", nameSpace: node3), "/wg/foo/bar")

        // global names

        XCTAssertEqual(resolve(name: "/bar", nameSpace: node1), "/bar")
        XCTAssertEqual(resolve(name: "/bar", nameSpace: node2), "/bar")
        XCTAssertEqual(resolve(name: "/foo/bar", nameSpace: node3), "/foo/bar")


        // private names

        XCTAssertEqual(resolve(name: "~bar", nameSpace: node1), "/node1/bar")
        XCTAssertEqual(resolve(name: "~bar", nameSpace: node2), "/wg/node2/bar")
        XCTAssertEqual(resolve(name: "~foo/bar", nameSpace: node3), "/wg/node3/foo/bar")

        // Empty name


        XCTAssertEqual(resolve(name: "", nameSpace: ""), "/")

        let name = resolve(name: "~name", nameSpace: "test")
        XCTAssertEqual(name, "test/name")

        XCTAssertEqual(namespace(name: "bar"), "/")
        XCTAssertEqual(namespace(name: "foo/bar"), "/foo/")
        XCTAssertEqual(namespace(name: ""), "/")
        XCTAssertEqual(namespace(name: "/foo/bar/"), "/foo/")
        XCTAssertEqual(namespace(name: "/foo/bar/node"), "/foo/bar/")

    }

    func testParameterServer() {
        let rm = RegistrationManager()
        let ps = ParameterServer<Int>(reg_manager: rm)

        let caller = Caller(id: "test", api: "api")

        let _ = ps.subscribe(parameter: "/param", node: caller)
        let _ = ps.subscribe(parameter: "/par", node: caller)


        ps.set(param: "/param", value: 23, notfiy: notify(updates:))
        ps.set(param: "par/am", value: 34, notfiy: notify(updates:))
        ps.set(param: "par/ma", value: 34, notfiy: notify(updates:))

        let names = ps.parameters.getNames().sorted()
        XCTAssertEqual(names, ["/par/am","/par/ma","/param"])

        ps.set(param: "par", value: 99, notfiy: notify(updates:))
        let names2 = ps.parameters.getNames().sorted()
        XCTAssertEqual(names2, ["/par","/param"])

        let value = ps.getValueFor(param: "/par")?.values
        XCTAssertEqual(value, 99)

    }

    func testUpdates() {
        let rm = RegistrationManager()
        let ps = ParameterServer<Int>(reg_manager: rm)
        let n1 = Caller(id: "/node1", api: "api")
        let n2 = Caller(id: "/node2", api: "api")
        let n3 = Caller(id: "/node3", api: "api")
        rm.register(parameterSubscriber: n1, parameter: "/foo/bar")
        rm.register(parameterSubscriber: n2, parameter: "/foo/bar")
        rm.register(parameterSubscriber: n3, parameter: "~foo")

        guard let par = ps.set(param: "foo/bar", value: 0) else {
            XCTFail()
            return
        }

        let updates = ps.computeUpdates(key: par, param_value: 0)
        let u = updates.map { $0.subscriber.id }
        XCTAssertEqual(u.sorted(), ["/node1","/node2"])

        guard let par2 = ps.set(param: "/node3/foo", value: 0) else {
            XCTFail()
            return
        }

        let updates2 = ps.computeUpdates(key: par2, param_value: 0)
        let u2 = updates2.map { $0.subscriber.id }
        XCTAssertEqual(u2.sorted(), ["/node3"])

        // This will delete /foo/bar

        guard let par3 = ps.set(param: "/foo", value: 0) else {
            XCTFail()
            return
        }

        let updates3 = ps.computeUpdates(key: par3, param_value: 0)
        let u3 = updates3.map { $0.subscriber.id }
        XCTAssertEqual(u3.sorted(), ["/node1","/node2"])

    }

    func testUpdates2() {
        let rm = RegistrationManager()
        let ps = ParameterServer<Int>(reg_manager: rm)
        let n1 = Caller(id: "/node1", api: "api")
        rm.register(parameterSubscriber: n1, parameter: "/foo")
        guard let p = ps.set(param: "/foo", value: 0) else {
            XCTFail()
            return
        }

        let up1 = ps.computeUpdates(key: p, param_value: 0)
        XCTAssertEqual(up1.first?.key, "/foo")
        guard let par = ps.set(param: "/foo/bar", value: 0) else {
            XCTFail()
            return
        }
        let up2 = ps.computeUpdates(key: par, param_value: 0)
        XCTAssertEqual(up2.first?.key, "/foo/bar")
    }


    func testComputeAllKeys() {
        let rm = RegistrationManager()
        let ps = ParameterServer<XmlRpcValue>(reg_manager: rm)
        let n1 = Caller(id: "/node1", api: "api")
        let n2 = Caller(id: "/node2", api: "api")
        let n3 = Caller(id: "/node3", api: "api")
        rm.register(parameterSubscriber: n1, parameter: "/foo")
        rm.register(parameterSubscriber: n2, parameter: "foo")
        rm.register(parameterSubscriber: n3, parameter: "~foo")

        let dict: [String: Any] = ["foo" : 3, "web": ["par" : 4, "t" : ["g":8,"k":9]] ]
        let value = XmlRpcValue(any: dict)
        var all = [String]()
        ps.computeAllKeys(param: "/foo", value: value, all_keys: &all)
        XCTAssertEqual(all.sorted(), ["/foo/foo/",
                                      "/foo/web/",
                                      "/foo/web/par/",
                                      "/foo/web/t/",
                                      "/foo/web/t/g/",
                                      "/foo/web/t/k/"])
        
    }
    
    func testChangeSibling() {
        let rm = RegistrationManager()
        let ps = ParameterServer<XmlRpcValue>(reg_manager: rm)
        let val = XmlRpcValue(any: ["g": 4, "k" : 7, "level": ["one": 1, "two": 2]])
        let _ = ps.set(param: "/foo/web/t", value: val)
        let _ = ps.set(param: "/foo/bew/t", value: val)
        let t = ps.getValueFor(param: "/foo/web/t")
        XCTAssertNotNil(t)
        XCTAssertEqual(t?.values, val)
        let g = ps.getValueFor(param: "/foo/web/t/g")
        XCTAssertNotNil(g)
        XCTAssertEqual(g?.values.int, 4)
        let _ = ps.set(param: "/foo/web/t/g", value: XmlRpcValue(any: 42))
        let k = ps.getValueFor(param: "/foo/web/t/k")
        XCTAssertNotNil(k)
        XCTAssertEqual(k?.values.int, 7)
    }

    func testXmlRpcIteration() {
        let v = XmlRpcValue(anyArray: [1,2,3,4,5])
        XCTAssertEqual(v.count, 5)
        for k in v {
            print(k)
        }
        let v1 = XmlRpcValue(str: "string")
        XCTAssertEqual(v1.count, 1)
        for k in v1 {
            print(k)
        }
        let v2 = XmlRpcValue(any: ["hej": 1, "då": 3])
        XCTAssertEqual(v2.count, 2)
        for k in v2 {
            print(k)
        }

    }

    func testMultiMap() {
        var map = Multimap<Int,Int>()
        XCTAssert(map.isEmpty)
        map.insert(value: 2, forKey: 1)
        map.insert(value: 3, forKey: 1)
        map.insert(value: 4, forKey: 1)
        map.insert(value: 4, forKey: 2)
        map.insert(value: 4, forKey: 3)
        XCTAssert(map.contains(value: 2, forKey: 1))
        XCTAssert(map.contains(value: 3, forKey: 1))
        XCTAssert(map.contains(value: 4, forKey: 1))
        XCTAssert(map.contains(value: 4, forKey: 2))
        XCTAssert(map.contains(value: 4, forKey: 3))
        XCTAssert(map.contains(key: 1))
        XCTAssert(map.contains(key: 2))
        XCTAssert(map.contains(key: 3))
        XCTAssertEqual(map.keyCount, 3)
        XCTAssertFalse(map.isEmpty)
        map.removeValue(4, forKey: 3)
        XCTAssertFalse(map.isEmpty)
        XCTAssertEqual(map.keyCount, 2)
        map.removeValuesForKey(1)
        XCTAssertFalse(map.isEmpty)
        XCTAssertEqual(map.keyCount, 1)
        map.removeAll()
        XCTAssert(map.isEmpty)
    }

}
