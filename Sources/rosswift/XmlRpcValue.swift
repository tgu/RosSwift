//
//  XmlRpcValue.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-05.
//

import Foundation

public final class XmlRpcValue {

    var value: Value

    enum Value: Equatable {
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
    }

    typealias BinaryData = [UInt8]
    typealias ValueArray = [XmlRpcValue]
    typealias ValueStruct = [String : XmlRpcValue]


    public init() {
        value = .invalid
    }

    init(str: String) {
        value = .string(str)
    }

    init(array: [XmlRpcValue]) {
        value = .array(array)
    }

    init(binary: [UInt8]) {
        value = .base64(binary)
    }

    init(any: Any) {
        switch any {
        case let b as Bool:
            value = .boolean(b)
        case let s as String:
            value = .string(s)
        case let i as UInt:
            value = .int(Int(i))
        case let i as Int32:
            value = .int(Int(i))
        case let i as Int:
            value = .int(i)
        case let d as Double:
            value = .double(d)
        case let f as Float32:
            value = .float(f)
        case let x as XmlRpcValue:
            value = x.value
        case let x as [String:Any]:
            var s = ValueStruct()
            x.forEach { (arg) in
                let (key, v) = arg
                s[key] = XmlRpcValue(any: v)
            }
            value = .struct(s)
        case let a as [Any]:
            value = XmlRpcValue(anyArray: a).value
        default:
            ROS_ERROR("switch failed")
            fatalError("switch failed")
        }
    }

    init(anyArray: [Any]) {
        let val = anyArray.map { XmlRpcValue(any: $0) }
        value = .array(val)
//        if val.count == 1 {
//            value = val.first!.value
//        } else {
//            value = .array(val)
//        }
    }

    func size() -> Int {
        switch value {
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

    var isArray : Bool {
        if case .array(_) = value {
            return true
        }
       return false
    }

    var isStruct: Bool {
        if case .struct(_) = value {
            return true
        }
        return false
    }

    var isString: Bool {
        if case .string(_) = value {
            return true
        }
        return false
    }

    subscript(i: Int) -> XmlRpcValue {
        if case .array(let a) = value {
            return a[i]
        }
        if i == 0 {
            return self
        }
        fatalError()
    }

    var uncertainString: String? {
        switch value {
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

    var string: String {
        switch value {
        case .string(let s):
            return s
        case .array(let a):
            if a.count == 1 {
                return a[0].string
            }
        default:
            fatalError()
        }
        return ""
    }

    var `struct`: ValueStruct? {
        if case .struct(let s) = value {
            return s
        }
        return nil
    }

    var int: Int? {
        if case .int(let i) = value {
            return i
        }
        return nil
    }

    var bool: Bool? {
        if case .boolean(let b) = value {
            return b
        }
        return nil
    }


    var date: Date? {
        if case .datetime(let d) = value {
            return d
        }
        return nil
    }



    func clear() {
        invalidate()
    }

    func invalidate() {
        value = .invalid
    }

    func valid() -> Bool {
        if case .invalid = value {
            return false
        }
        return true
    }

    func toXml() -> String {
        switch value {
        case .boolean(let b):
            return Tags.bool(b)
        case .int(let i):
            return Tags.int(i)
        case .double(let d):
            return Tags.double(d)
        case .float(let f):
            return Tags.float32(f)
        case .string(let s):
            return Tags.string(s)
        case .`struct`(let s):
            return s.xml
        case .array(let a):
            let xml = a.reduce("", {$0 + $1.toXml()})
            return Tags.array(xml)
        default:
            fatalError()
        }
    }


    @discardableResult
    func get<T: Numeric>(val: inout [T]) -> Bool {
        switch value {
        case .array(let a):
            let vec = a.compactMap { (v) -> T? in
                var d : T = 0
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
        switch value {
        case .array(let a):
            let vec = a.compactMap { (v) -> T? in
                var d : T = ""
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
        let arr = vec.compactMap{ d -> T? in
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
        let arr = vec.compactMap{ d -> Bool? in
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
        let arr = vec.compactMap{ d -> String? in
            return d.uncertainString 
        }
        guard arr.count == vec.count else {
            return nil
        }
        return arr
    }

    func getMap(_ m: [String:XmlRpcValue]) -> [String:String]? {
        var nm = [String:String]()
        for k in m {
            guard let s = k.value.uncertainString else {
                return nil
            }
            nm[k.key] = s
        }
        return nm
    }

    func getMap<T: Numeric>(_ m: [String:XmlRpcValue]) -> [String:T]? {
        var nm = [String:T]()
        for k in m {
            var d : T = 0
            guard k.value.get(val: &d) else {
                return nil
            }
            nm[k.key] = d
        }
        return nm
    }

    func getBoolMap(_ m: [String:XmlRpcValue]) -> [String:Bool]? {
        var nm = [String:Bool]()
        for k in m {
            var d : Double = 0
            guard k.value.get(val: &d) else {
                return nil
            }
            nm[k.key] = d != 0
        }
        return nm
    }




    func get<T>(val: inout T) -> Bool {
        switch value {
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
            guard let vec : [String] = getArray(v) else {
                return false
            }
            val = vec as! T
        case .array(let v) where T.self == [Double].self:
            guard let vec : [Double] = getArray(v) else {
                return false
            }
            val = vec as! T
        case .array(let v) where T.self == [Float32].self:
            guard let vec : [Float32] = getArray(v) else {
                return false
            }
            val = vec as! T
        case .array(let v) where T.self == [Int].self:
            guard let vec : [Int] = getArray(v) else {
                return false
            }
            val = vec as! T

        case .array(let v) where T.self == [Bool].self:
            guard let vec : [Bool] = getArray(v) else {
                return false
            }
            val = vec as! T

        case .`struct`(let v as T):
            val = v
        case .`struct`(let v) where T.self == [String:String].self:
            guard let map  = getMap(v) else {
                return false
            }
            val = map as! T
        case .`struct`(let v) where T.self == [String:Double].self:
            guard let map : [String:Double] = getMap(v) else {
                return false
            }
            val = map as! T
        case .`struct`(let v) where T.self == [String:Float32].self:
            guard let map : [String:Float32] = getMap(v) else {
                return false
            }
            val = map as! T
        case .`struct`(let v) where T.self == [String:Int].self:
            guard let map : [String:Int] = getMap(v) else {
                return false
            }
            val = map as! T

        case .`struct`(let v) where T.self == [String:Bool].self:
            guard let map  = getBoolMap(v) else {
                return false
            }
            val = map as! T

        default:
            ROS_DEBUG("Could not get \(String(describing: T.self)) value from \(value)")
            return false
        }
        return true
    }

    func getType() -> Value {
        return value
    }

    func hasMember(_ name: String) -> Bool {
        if case .`struct`(let v) = value {
            return v.keys.contains(name)
        }
        return false
    }

    subscript(_ name: String) -> XmlRpcValue? {
        if case .`struct`(let v) = value {
            return v[name]
        }
        return nil
    }

}

extension XmlRpcValue: Equatable {
    public static func == (lhs: XmlRpcValue, rhs: XmlRpcValue) -> Bool {
        return lhs.value == rhs.value
    }
}


extension XmlRpcValue: CustomStringConvertible {
    public var description: String {
        switch value {
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

enum Tags : String {
    case METHODNAME_TAG = "<methodName>"
    case METHODNAME_ETAG = "</methodName>"
    case PARAMS_TAG = "<params>"
    case PARAMS_ETAG = "</params>"
    case PARAM_TAG = "<param>"
    case PARAM_ETAG =  "</param>"
    case FAULT_TAG = "<fault>"

    case VALUE_TAG     = "<value>"
    case VALUE_ETAG    = "</value>"

    case BOOLEAN_TAG   = "<boolean>"
    case BOOLEAN_ETAG  = "</boolean>"
    case DOUBLE_TAG    = "<double>"
    case DOUBLE_ETAG   = "</double>"
    case INT_TAG       = "<int>"
    case I4_TAG        = "<i4>"
    case I4_ETAG       = "</i4>"
    case STRING_TAG    = "<string>"
    case STRING_ETAG    = "</string>"
    case DATETIME_TAG  = "<dateTime.iso8601>"
    case DATETIME_ETAG = "</dateTime.iso8601>"
    case BASE64_TAG    = "<base64>"
    case BASE64_ETAG   = "</base64>"

    case ARRAY_TAG     = "<array>"
    case DATA_TAG      = "<data>"
    case DATA_ETAG     = "</data>"
    case ARRAY_ETAG    = "</array>"

    case STRUCT_TAG    = "<struct>"
    case MEMBER_TAG    = "<member>"
    case NAME_TAG      = "<name>"
    case NAME_ETAG     = "</name>"
    case MEMBER_ETAG   = "</member>"
    case STRUCT_ETAG   = "</struct>"

    static func member(_ name: String, value: String) -> String {
        return "\(MEMBER_TAG.rawValue)\(NAME_TAG.rawValue)\(name)\(NAME_ETAG.rawValue)\(value)\(MEMBER_ETAG.rawValue)"
    }

    static func bool(_ value: Bool) -> String {
        return "\(VALUE_TAG.rawValue)\(BOOLEAN_TAG.rawValue)\(value ? "1" : "0")\(BOOLEAN_ETAG.rawValue)\(VALUE_ETAG.rawValue)"
    }

    static func int(_ value: Int) -> String {
        return "\(VALUE_TAG.rawValue)\(I4_TAG.rawValue)\(value)\(I4_ETAG.rawValue)\(VALUE_ETAG.rawValue)"
    }

    static func double(_ value: Double) -> String {
        return "\(VALUE_TAG.rawValue)\(DOUBLE_TAG.rawValue)\(value)\(DOUBLE_ETAG.rawValue)\(VALUE_ETAG.rawValue)"
    }

    static func float32(_ value: Float32) -> String {
        return "\(VALUE_TAG.rawValue)\(DOUBLE_TAG.rawValue)\(value)\(DOUBLE_ETAG.rawValue)\(VALUE_ETAG.rawValue)"
    }

    static func string(_ value: String) -> String {
        return "\(VALUE_TAG.rawValue)\(STRING_TAG.rawValue)\(value)\(STRING_ETAG.rawValue)\(VALUE_ETAG.rawValue)"
    }

    static func `struct`(_ value: String) -> String {
        return "\(VALUE_TAG.rawValue)\(STRUCT_TAG.rawValue)\(value)\(STRUCT_ETAG.rawValue)\(VALUE_ETAG.rawValue)"
    }

    static func array(_ value: String) -> String {
        return "\(VALUE_TAG.rawValue)\(ARRAY_TAG.rawValue)\(DATA_TAG.rawValue)\(value)\(DATA_ETAG.rawValue)\(ARRAY_ETAG.rawValue)\(VALUE_ETAG.rawValue)"
    }

}

extension Dictionary where Value: XmlRpcValue {
    var xml: String {
        let a = self.map { (arg) -> String in
            let (key, value) = arg
            return Tags.member((key as! String), value: value.toXml())
        }.joined()
        return Tags.`struct`(a)
    }
}
