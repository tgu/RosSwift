//
//  XmlRpcValue.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-05.
//

import Foundation

protocol ConvertableToXml {
    func toXml() -> String
}

/// Remote Procedure Call (RPC) Object

public enum XmlRpcValue: Equatable, ConvertableToXml {
    case invalid
    case boolean(Bool)
    case int(Int)
    case float(Float32)
    case double(Double)
    case string(String)
    case datetime(Date)
    case base64(BinaryData)
    case array(ValueArray)
    case `struct`(ValueStruct)

    public typealias BinaryData = [UInt8]
    public typealias ValueArray = [XmlRpcValue]
    public typealias ValueStruct = [String: XmlRpcValue]

    public init() {
        self = .invalid
    }

    public init(str: String) {
        self = .string(str)
    }

    public init(array: [XmlRpcValue]) {
        self = .array(array)
    }

    public init(binary: [UInt8]) {
        self = .base64(binary)
    }

    public init(any: Any) {
        switch any {
        case let b as Bool:
            self = .boolean(b)
        case let s as String:
            self = .string(s)
        case let i as UInt:
            self = .int(Int(i))
        case let i as Int32:
            self = .int(Int(i))
        case let i as Int:
            self = .int(i)
        case let d as Double:
            self = .double(d)
        case let f as Float32:
            self = .float(f)
        case let x as XmlRpcValue:
            self = x
        case let x as ValueStruct:
            self = .struct(x)
        case let x as (String, XmlRpcValue):
            var s = ValueStruct()
            s[x.0] = x.1
            self = .struct(s)
        case let x as [String: Any]:
            var s = ValueStruct()
            x.forEach { arg in
                let (key, v) = arg
                s[key] = XmlRpcValue(any: v)
            }
            self = .struct(s)
        case let a as [Any]:
            self = XmlRpcValue(anyArray: a)
        default:
            fatalError("switch failed")
        }
    }

    public init(anyArray: [Any]) {
        let val = anyArray.map { XmlRpcValue(any: $0) }
        self = .array(val)
    }

    public var count: Int {
        switch self {
        case .array(let a):
            return a.count
        case .`struct`(let s):
            return s.count
        default:
            return 1
        }
    }

    public func size() -> Int {
        switch self {
        case .string(let s):
            return s.count
        case .base64(let b):
            return b.count
        case .array(let a):
            return a.count
        case .`struct`(let s):
            return s.count
        default:
            return 1
        }
    }

    public var isArray: Bool {
        if case .array(_) = self {
            return true
        }
        return false
    }

    public var isStruct: Bool {
        if case .struct(_) = self {
            return true
        }
        return false
    }

    public var isString: Bool {
        if case .string(_) = self {
            return true
        }
        return false
    }

    public subscript(i: Int) -> XmlRpcValue {
        if case .array(let a) = self {
            return a[i]
        }
        if case .`struct`(let s) = self {
            let elem = Array(s)[i]
            return XmlRpcValue(any: elem)
        }
        if i == 0 {
            return self
        }
        fatalError("subscript accesing \(i)")
    }

    var uncertainString: String? {
        switch self {
        case .string(let s):
            return s
        case .array(let a):
            if a.count == 1 {
                return a[0].string
            }
        default:
            return nil
        }
        return nil
    }

    public var string: String {
        switch self {
        case .string(let s):
            return s
        case .array(let a):
            if a.count == 1 {
                return a[0].string
            }
        default:
            fatalError("not a string \(self)")
        }
        return ""
    }

    public var dictionary: ValueStruct? {
        if case .struct(let s) = self {
            return s
        }
        return nil
    }

    public var array: ValueArray? {
        if case .array(let s) = self {
            return s
        }
        return nil
    }

    var int: Int? {
        if case .int(let i) = self {
            return i
        }
        return nil
    }

    var bool: Bool? {
        if case .boolean(let b) = self {
            return b
        }
        return nil
    }

    var date: Date? {
        if case .datetime(let d) = self {
            return d
        }
        return nil
    }

    mutating func clear() {
        invalidate()
    }

    mutating func invalidate() {
        self = .invalid
    }

    public func valid() -> Bool {
        if case .invalid = self {
            return false
        }
        return true
    }

    public func toXml() -> String {
        switch self {
        case .boolean(let b):
            return Tags.boolXml(b)
        case .int(let i):
            return Tags.intXml(i)
        case .double(let d):
            return Tags.doubleXml(d)
        case .float(let f):
            return Tags.float32xml(f)
        case .string(let s):
            return Tags.stringXml(s)
        case .`struct`(let s):
            return s.xml
        case .array(let a):
            let xml = a.reduce("", { $0 + $1.toXml() })
            return Tags.arrayXml(xml)
        default:
            fatalError("Could not convert to xml: \(self)")
        }
    }

    @discardableResult
    func get<T: Numeric>(val: inout [T]) -> Bool {
        switch self {
        case .array(let a):
            let vec = a.compactMap { v -> T? in
                var d: T = 0
                if v.get(val: &d) {
                    return d
                }
                return nil
            }
            val = vec
            return vec.count == a.count
        default:
            return false
        }
    }

    @discardableResult
    func get<T: StringProtocol>(val: inout [T]) -> Bool {
        switch self {
        case .array(let a):
            let vec = a.compactMap { v -> T? in
                var d: T = ""
                if v.get(val: &d) {
                    return d
                }
                return nil
            }
            val = vec
            return vec.count == a.count

        default:
            return false
        }
    }

    func getArray<T: Numeric>(_ vec: [XmlRpcValue]) -> [T]? {
        let arr = vec.compactMap { d -> T? in
            var c: T = 0
            guard d.get(val: &c) else {
                return nil
            }
            return c
        }
        guard arr.count == vec.count else {
            return nil
        }
        return arr
    }

    func getArray(_ vec: [XmlRpcValue]) -> [Bool]? {
        let arr = vec.compactMap { d -> Bool? in
            var c = 0.0
            guard d.get(val: &c) else {
                return nil
            }
            return c != 0
        }
        guard arr.count == vec.count else {
            return nil
        }
        return arr
    }

    func getArray(_ vec: [XmlRpcValue]) -> [String]? {
        let arr = vec.compactMap { d -> String? in
            d.uncertainString
        }
        guard arr.count == vec.count else {
            return nil
        }
        return arr
    }

    func getMap(_ m: [String: XmlRpcValue]) -> [String: String]? {
        var map = [String: String]()
        for k in m {
            guard let s = k.value.uncertainString else {
                return nil
            }
            map[k.key] = s
        }
        return map
    }

    func getMap<T: Numeric>(_ m: [String: XmlRpcValue]) -> [String: T]? {
        var map = [String: T]()
        for k in m {
            var d: T = 0
            guard k.value.get(val: &d) else {
                return nil
            }
            map[k.key] = d
        }
        return map
    }

    func getBoolMap(_ m: [String: XmlRpcValue]) -> [String: Bool]? {
        var map = [String: Bool]()
        for k in m {
            var d: Double = 0
            guard k.value.get(val: &d) else {
                return nil
            }
            map[k.key] = d != 0
        }
        return map
    }

    func get<T>(val: inout T) -> Bool {
        switch self {
        case .invalid:
            return false
        case .boolean(let b as T):
            val = b
        case .boolean(let b) where T.self == Int.self:
            val = (b ? 1 : 0) as! T
        case .boolean(let b) where T.self == Float32.self:
            val = (b ? Float32(1.0) : Float32(0.0)) as! T
        case .boolean(let b) where T.self == Double.self:
            val = (b ? 1.0 : 0.0) as! T
        case .int(let i) where T.self == Bool.self:
            val = (i != 0) as! T
        case .int(let i as T):
            val = i
        case .int(let i) where T.self == Double.self:
            val = Double(i) as! T
        case .int(let i) where T.self == Float32.self:
            val = Float32(i) as! T
        case .double(let d as T):
            val = d
        case .double(let d) where T.self == Float32.self:
            val = Float32(d) as! T
        case .double(let d) where T.self == Int.self:
            val = (Int(exactly: round(d)) ?? 0) as! T
        case .string(let s as T):
            val = s
        case .datetime(let d as T):
            val = d
        case .base64(let b as T):
            val = b
        case .array(let v) where T.self == [String].self:
            guard let vec: [String] = getArray(v) else {
                return false
            }
            val = vec as! T
        case .array(let v) where T.self == [Double].self:
            guard let vec: [Double] = getArray(v) else {
                return false
            }
            val = vec as! T
        case .array(let v) where T.self == [Float32].self:
            guard let vec: [Float32] = getArray(v) else {
                return false
            }
            val = vec as! T
        case .array(let v) where T.self == [Int].self:
            guard let vec: [Int] = getArray(v) else {
                return false
            }
            val = vec as! T

        case .array(let v) where T.self == [Bool].self:
            guard let vec: [Bool] = getArray(v) else {
                return false
            }
            val = vec as! T

        case .`struct`(let v as T):
            val = v
        case .`struct`(let v) where T.self == [String: String].self:
            guard let map = getMap(v) else {
                return false
            }
            val = map as! T
        case .`struct`(let v) where T.self == [String: Double].self:
            guard let map: [String: Double] = getMap(v) else {
                return false
            }
            val = map as! T
        case .`struct`(let v) where T.self == [String: Float32].self:
            guard let map: [String: Float32] = getMap(v) else {
                return false
            }
            val = map as! T
        case .`struct`(let v) where T.self == [String: Int].self:
            guard let map: [String: Int] = getMap(v) else {
                return false
            }
            val = map as! T

        case .`struct`(let v) where T.self == [String: Bool].self:
            guard let map = getBoolMap(v) else {
                return false
            }
            val = map as! T

        default:
            ROS_ERROR("Could not get \(String(describing: T.self)) value from \(self)")
            return false
        }
        return true
    }

    //    func getType() -> Value {
    //        return value
    //    }
    //
    func hasMember(_ name: String) -> Bool {
        if case .`struct`(let v) = self {
            return v.keys.contains(name)
        }
        return false
    }

    subscript(_ name: String) -> XmlRpcValue? {
        if case .`struct`(let v) = self {
            return v[name]
        }
        return nil
    }

}

extension XmlRpcValue: CustomStringConvertible {
    public var description: String {
        switch self {
        case .invalid:
            return "invalid"
        case .boolean(let b):
            return b ? "true" : "false"
        case .int(let i):
            return "\(i)"
        case .double(let d):
            return "\(d)"
        case .float(let f):
            return "\(f)"
        case .string(let s):
            return s
        case .datetime(let d):
            return String(describing: d)
        case .base64(let b):
            return String(describing: b)
        case .array(let a):
            return "\(a)"
        case .`struct`(let s):
            return "\(s)"
        }
    }
}

public enum Tags: String {
    case methodname =       "<methodName>"
    case endMethodname =    "</methodName>"
    case params =           "<params>"
    case endParams =        "</params>"
    case param =            "<param>"
    case endParam =         "</param>"
    case fault =            "<fault>"

    case value =            "<value>"
    case endValue =         "</value>"

    case boolean =          "<boolean>"
    case endBoolean  =      "</boolean>"
    case double =           "<double>"
    case endDouble =        "</double>"
    case int =              "<int>"
    case endInt =           "</int>"
    case i4Tag =            "<i4>"
    case i4ETag =           "</i4>"
    case string =           "<string>"
    case endString =        "</string>"
    case datetime =         "<dateTime.iso8601>"
    case endDatetime =      "</dateTime.iso8601>"
    case base64 =           "<base64>"
    case endBase64 =        "</base64>"

    case array =            "<array>"
    case data =             "<data>"
    case endData =          "</data>"
    case endArray =         "</array>"

    case structTag =        "<struct>"
    case member =           "<member>"
    case name =             "<name>"
    case endName =          "</name>"
    case endMember =        "</member>"
    case endStruct =        "</struct>"

    static func memberXml(_ memberName: String, value: String) -> String {
        return "\(member.rawValue)\(name.rawValue)\(memberName)\(endName.rawValue)\(value)\(endMember.rawValue)"
    }

    static func boolXml(_ val: Bool) -> String {
        return "\(value.rawValue)\(boolean.rawValue)\(val ? "1" : "0")\(endBoolean.rawValue)\(endValue.rawValue)"
    }

    static func intXml(_ val: Int) -> String {
        return "\(value.rawValue)\(i4Tag.rawValue)\(val)\(i4ETag.rawValue)\(endValue.rawValue)"
    }

    static func doubleXml(_ val: Double) -> String {
        return "\(value.rawValue)\(double.rawValue)\(val)\(endDouble.rawValue)\(endValue.rawValue)"
    }

    static func float32xml(_ val: Float32) -> String {
        return "\(value.rawValue)\(double.rawValue)\(val)\(endDouble.rawValue)\(endValue.rawValue)"
    }

    static func stringXml(_ val: String) -> String {
        return "\(value.rawValue)\(string.rawValue)\(val)\(endString.rawValue)\(endValue.rawValue)"
    }

    static func structXml(_ val: String) -> String {
        return "\(value.rawValue)\(structTag.rawValue)\(val)\(endStruct.rawValue)\(endValue.rawValue)"
    }

    static func arrayXml(_ val: String) -> String {
        return "\(value.rawValue)\(array.rawValue)\(data.rawValue)\(val)" +
        "\(endData.rawValue)\(endArray.rawValue)\(endValue.rawValue)"
    }

}

extension Dictionary where Value: ConvertableToXml {
    var xml: String {
        let a = self.map { arg -> String in
            let (key, value) = arg
            return Tags.memberXml((key as! String), value: value.toXml())
            }.joined()
        return Tags.structXml(a)
    }
}

extension XmlRpcValue: Collection {
    public func index(after i: Int) -> Int {
        return i + 1
    }

    public var startIndex: Int {
        return 0
    }

    public var endIndex: Int {
        return count
    }
}
