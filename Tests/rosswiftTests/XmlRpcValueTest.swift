//
//  XmlRpcValueTest.swift
//  rosswiftTests
//
//  Created by Thomas Gustafsson on 2018-10-31.
//

import XCTest
@testable import rpcobject

class XmlRpcValueTest: XCTestCase {

    func testStringArray() {
        let arr = ["a","b","c"]
        let a = XmlRpcValue(anyArray: arr)
        var sa = [String]()
        a.get(val: &sa)

        XCTAssertEqual(arr.count, sa.count)
        XCTAssertEqual(arr, sa)
    }

    func testDoubleArray() {
        let arr = [1.0,20.3,3.2]
        let a = XmlRpcValue(anyArray: arr)
        var sa = [Double]()
        a.get(val: &sa)

        XCTAssertEqual(arr.count, sa.count)
        XCTAssertEqual(arr, sa)
    }

    func testDoubleAsIntArray() {
        let arr = [1.0,20.3,3.2]
        let a = XmlRpcValue(anyArray: arr)
        var sa = [Int]()
        a.get(val: &sa)

        XCTAssertEqual(arr.count, sa.count)
        XCTAssertEqual([1,20,3], sa)
    }

    func testInvalid() {
        let x = XmlRpcValue()
        XCTAssertFalse(x.valid())
    }

    func testStringMap() {
        let map_s = ["a": "apple", "b": "blueberry", "c": "carrot"]
        let a = XmlRpcValue(any: map_s)
        var sa = [String:String]()
        _ = a.get(val: &sa)

        XCTAssertEqual(map_s, sa)

    }


}
