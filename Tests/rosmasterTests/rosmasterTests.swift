import Testing

@testable import rosmaster
@testable import rpcobject


extension Int: ArrayConstructable, @retroactive Collection {
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

@Suite("Rosmaster tests")
struct RosmasterTests {

    @Test func longName() {
        let r = RadixTree<Int>()
        #expect(r.insert("top/name/space/with/variable/ens", value: 456) != nil)
        #expect(r.height == 6)
        let g1 = r.find("top/name/space/with/variable")
        let g2 = r.find("top/name/space/with")
        let g3 = r.find("top/name/space")
        let g4 = r.find("top/name")
        let g5 = r.find("top")
        let g6 = r.find("")
        let g7 = r.find("top/name/spacer")

        #expect(g1)
        #expect(g2)
        #expect(g3)
        #expect(g4)
        #expect(g5)
        #expect(g6)
        #expect(!g7)
    }

    @Test func getLongName() {
        let r = RadixTree<Int>()
        #expect(r.insert("top/name/space/with/variable/ens", value: 456) != nil)
        #expect(r.insert("top/name/space/with/var", value: 999) != nil)
        #expect(r.insert("top/name/here", value: 123) != nil)
        #expect(r.height == 6)
        let g1 = r.get("top/name/space/with/variable")
        let g2 = r.get("top/name/space/with")
        let g3 = r.get("top/name/space")
        let g4 = r.get("top/name")
        let g5 = r.get("top")
        let g6 = r.get("")
        let g7 = r.get("top/name/spacer")

        #expect(g1 != nil)
        #expect(g2 != nil)
        #expect(g3 != nil)
        #expect(g4 != nil)
        #expect(g5 != nil)
        #expect(g6 != nil)
        #expect(g7 == nil)

        #expect(g1?.children.count == 1)
        #expect(g2?.children.count == 2)
        #expect(g4?.children.count == 2)
    }

    @Test func parameters() {
        let r = RadixTree<Int>()

        #expect(r.insert("rom/nus", value: 1) != nil)

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
        #expect(!r.find("courts")) // false
        #expect(!r.find("r")) // true
        #expect(!r.find("ro")) // true
        #expect(r.find("rom")) // true
        #expect(r.find("rom")) // true
        #expect(r.find("top/name/space/with/variable/ens")) // true
        #expect(!r.find("roman")) // true
        #expect(r.find("rom/ane")) // true
        #expect(!r.find("romans")) // false
        #expect(!r.find("steve")) // true
        print("\n\nREMOVE TESTS")

        #expect(r.remove("c") == nil)
        #expect(r.remove("rub") != nil)
        #expect(r.find("rom/ane")) // true
        r.printTree()
        #expect(r.remove("stevenson") == nil)
        #expect(r.find("rom")) // true
        #expect(r.remove("rom") != nil)
        #expect(!r.find("rom/ane")) // true
        r.printTree()
    }

    private func notify(updates: [Update<Int>]) {
        updates.forEach { print($0) }
    }

    @Test func resolveNames() {
        let node1 = "/node1"
        let node2 = "/wg/node2"
        let node3 = "/wg/node3"

        // relative names

        #expect(resolve(name: "bar", nameSpace: node1) == "/bar")
        #expect(resolve(name: "bar", nameSpace: node2) == "/wg/bar")
        #expect(resolve(name: "foo/bar", nameSpace: node3) == "/wg/foo/bar")

        // global names

        #expect(resolve(name: "/bar", nameSpace: node1) == "/bar")
        #expect(resolve(name: "/bar", nameSpace: node2) == "/bar")
        #expect(resolve(name: "/foo/bar", nameSpace: node3) == "/foo/bar")

        // private names

        #expect(resolve(name: "~bar", nameSpace: node1) == "/node1/bar")
        #expect(resolve(name: "~bar", nameSpace: node2) == "/wg/node2/bar")
        #expect(resolve(name: "~foo/bar", nameSpace: node3) == "/wg/node3/foo/bar")

        // Empty name

        #expect(resolve(name: "", nameSpace: "") == "/")

        let name = resolve(name: "~name", nameSpace: "test")
        #expect(name == "test/name")

        #expect(namespace(name: "bar") == "/")
        #expect(namespace(name: "foo/bar") == "/foo/")
        #expect(namespace(name: "") == "/")
        #expect(namespace(name: "/foo/bar/") == "/foo/")
        #expect(namespace(name: "/foo/bar/node") == "/foo/bar/")
    }

    @Test func parameterServer() {
        let rm = RegistrationManager()
        let ps = ParameterServer<Int>(reg_manager: rm)

        let caller = Caller(id: "test", api: "api")

        let _ = ps.subscribe(parameter: "/param", node: caller)
        let _ = ps.subscribe(parameter: "/par", node: caller)

        ps.set(param: "/param", value: 23, notfiy: notify(updates:))
        ps.set(param: "par/am", value: 34, notfiy: notify(updates:))
        ps.set(param: "par/ma", value: 34, notfiy: notify(updates:))

        let names = ps.parameters.getNames().sorted()
        #expect(names == ["/par/am","/par/ma","/param"])

        ps.set(param: "par", value: 99, notfiy: notify(updates:))
        let names2 = ps.parameters.getNames().sorted()
        #expect(names2 == ["/par","/param"])

        let value = ps.getValueFor(param: "/par")?.values
        #expect(value == 99)
    }

    @Test func updates() throws {
        let rm = RegistrationManager()
        let ps = ParameterServer<Int>(reg_manager: rm)
        let n1 = Caller(id: "/node1", api: "api")
        let n2 = Caller(id: "/node2", api: "api")
        let n3 = Caller(id: "/node3", api: "api")
        rm.register(parameterSubscriber: n1, parameter: "/foo/bar")
        rm.register(parameterSubscriber: n2, parameter: "/foo/bar")
        rm.register(parameterSubscriber: n3, parameter: "~foo")

        let par = try #require(ps.set(param: "foo/bar", value: 0))

        let updates = ps.computeUpdates(key: par, param_value: 0)
        let u = updates.map { $0.subscriber.id }
        #expect(u.sorted() == ["/node1","/node2"])

        let par2 = try #require(ps.set(param: "/node3/foo", value: 0))

        let updates2 = ps.computeUpdates(key: par2, param_value: 0)
        let u2 = updates2.map { $0.subscriber.id }
        #expect(u2.sorted() == ["/node3"])

        // This will delete /foo/bar

        let par3 = try #require(ps.set(param: "/foo", value: 0))

        let updates3 = ps.computeUpdates(key: par3, param_value: 0)
        let u3 = updates3.map { $0.subscriber.id }
        #expect(u3.sorted() == ["/node1","/node2"])
    }

    @Test func updates2() throws {
        let rm = RegistrationManager()
        let ps = ParameterServer<Int>(reg_manager: rm)
        let n1 = Caller(id: "/node1", api: "api")
        rm.register(parameterSubscriber: n1, parameter: "/foo")
        let p = try #require(ps.set(param: "/foo", value: 0))

        let up1 = ps.computeUpdates(key: p, param_value: 0)
        #expect(up1.first?.key == "/foo")
        let par = try #require(ps.set(param: "/foo/bar", value: 0))
        let up2 = ps.computeUpdates(key: par, param_value: 0)
        #expect(up2.first?.key == "/foo/bar")
    }

    @Test func computeAllKeys() {
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
        #expect(all.sorted() == ["/foo/foo/",
                                 "/foo/web/",
                                 "/foo/web/par/",
                                 "/foo/web/t/",
                                 "/foo/web/t/g/",
                                 "/foo/web/t/k/"])
    }

    @Test func changeSibling() {
        let rm = RegistrationManager()
        let ps = ParameterServer<XmlRpcValue>(reg_manager: rm)
        let val = XmlRpcValue(any: ["g": 4, "k" : 7, "level": ["one": 1, "two": 2]])
        let _ = ps.set(param: "/foo/web/t", value: val)
        let _ = ps.set(param: "/foo/bew/t", value: val)
        let t = ps.getValueFor(param: "/foo/web/t")
        #expect(t != nil)
        #expect(t?.values == val)
        let g = ps.getValueFor(param: "/foo/web/t/g")
        #expect(g != nil)
        #expect(g?.values.int == 4)
        let _ = ps.set(param: "/foo/web/t/g", value: XmlRpcValue(any: 42))
        let k = ps.getValueFor(param: "/foo/web/t/k")
        #expect(k != nil)
        #expect(k?.values.int == 7)
    }

    @Test func xmlRpcIteration() {
        let v = XmlRpcValue(anyArray: [1,2,3,4,5])
        #expect(v.count == 5)
        for k in v {
            print(k)
        }
        let v1 = XmlRpcValue(str: "string")
        #expect(v1.count == 1)
        for k in v1 {
            print(k)
        }
        let v2 = XmlRpcValue(any: ["hej": 1, "då": 3])
        #expect(v2.count == 2)
        for k in v2 {
            print(k)
        }
    }

    @Test func multiMap() {
        var map = Multimap<Int,Int>()
        #expect(map.isEmpty)
        map.insert(value: 2, forKey: 1)
        map.insert(value: 3, forKey: 1)
        map.insert(value: 4, forKey: 1)
        map.insert(value: 4, forKey: 2)
        map.insert(value: 4, forKey: 3)
        #expect(map.contains(value: 2, forKey: 1))
        #expect(map.contains(value: 3, forKey: 1))
        #expect(map.contains(value: 4, forKey: 1))
        #expect(map.contains(value: 4, forKey: 2))
        #expect(map.contains(value: 4, forKey: 3))
        #expect(map.contains(key: 1))
        #expect(map.contains(key: 2))
        #expect(map.contains(key: 3))
        #expect(map.keyCount == 3)
        #expect(!map.isEmpty)
        map.removeValue(4, forKey: 3)
        #expect(!map.isEmpty)
        #expect(map.keyCount == 2)
        map.removeValuesForKey(1)
        #expect(!map.isEmpty)
        #expect(map.keyCount == 1)
        map.removeAll()
        #expect(map.isEmpty)
    }
}
