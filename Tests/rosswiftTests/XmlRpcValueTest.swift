//
//  XmlRpcValueTest.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-10-31.
//

import Testing
@testable import rpcobject

@Suite("XmlRpcValue tests")
struct XmlRpcValueTest {

    @Test func stringArray() {
        let arr = ["a","b","c"]
        let a = XmlRpcValue(anyArray: arr)
        var sa = [String]()
        a.get(val: &sa)

        #expect(arr.count == sa.count)
        #expect(arr == sa)
    }

    @Test func doubleArray() {
        let arr = [1.0,20.3,3.2]
        let a = XmlRpcValue(anyArray: arr)
        var sa = [Double]()
        a.get(val: &sa)

        #expect(arr.count == sa.count)
        #expect(arr == sa)
    }

    @Test func doubleAsIntArray() {
        let arr = [1.0,20.3,3.2]
        let a = XmlRpcValue(anyArray: arr)
        var sa = [Int]()
        a.get(val: &sa)

        #expect(arr.count == sa.count)
        #expect([1,20,3] == sa)
    }

    @Test func invalid() {
        let x = XmlRpcValue()
        #expect(!x.valid())
    }

    @Test func stringMap() {
        let map_s = ["a": "apple", "b": "blueberry", "c": "carrot"]
        let a = XmlRpcValue(any: map_s)
        var sa = [String:String]()
        _ = a.get(val: &sa)

        #expect(map_s == sa)
    }
}
