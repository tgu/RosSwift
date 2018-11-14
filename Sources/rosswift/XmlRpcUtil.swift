//
//  XmlRpcUtil.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-11-13.
//

import Foundation


extension XMLRPCClient {

    static func parseResponse(xml: String) -> XmlRpcValue? {
        guard let tagIndex = xml.range(of: METHODRESPONSE_TAG) else {
            ROS_ERROR("Invalid response - no methodResponse. Response:\n\(xml)")
            return nil
        }

        let result = XmlRpcValue()

        var data = xml.suffix(from: tagIndex.upperBound)
        if XmlRpcUtil.nextTagIs(tag: .PARAMS_TAG, xml: &data) && XmlRpcUtil.nextTagIs(tag: .PARAM_TAG, xml: &data) {
            if result.fromXML(xml: &data) {
                return result
            } else {
                ROS_ERROR("Invalid response value. Response: \(data)")
            }
        } else if XmlRpcUtil.nextTagIs(tag: .FAULT_TAG, xml: &data) {

        } else {
            ROS_ERROR("Invalid response - no param or fault tag. Response \(xml)")
        }


        // Expect either <params><param>... or <fault>...

        return nil
    }

}

enum XmlRpcUtil {

    static func nextTagIs(tag: Tags, xml: inout String.SubSequence) -> Bool {
        if xml.trimmingCharacters(in: .whitespacesAndNewlines).starts(with: tag.rawValue) {
            let tagIndex = xml.range(of: tag.rawValue)!
            xml = xml.suffix(from: tagIndex.upperBound)
            return true
        }

        return false
    }



    static func getNextTag(xml: inout String.SubSequence) -> String {
        guard let open = xml.firstIndex(of: "<"),
            let close = xml.firstIndex(of: ">"), close > open else {
                return ""
        }

        let tag = String(xml[open...close])
        xml = xml.suffix(from: close).dropFirst()
        return tag
    }

    static func findTag(tag: Tags, xml: inout String.SubSequence) -> Bool {
        if let range = xml.range(of: tag.rawValue) {
            xml = xml.suffix(from: range.upperBound)
            return true
        }
        return false
    }

    static func parseTag(from: Tags, to: Tags, xml: inout String.SubSequence) -> String {
        var xmlSeq = xml
        guard let fromRange = xmlSeq.range(of: from.rawValue) else {
            return ""
        }
        xmlSeq = xmlSeq.suffix(from: fromRange.upperBound)
        guard let toRange = xmlSeq.range(of: to.rawValue) else {
            return ""
        }
        xmlSeq = xmlSeq.prefix(upTo: toRange.lowerBound)
        xml = xml.suffix(from: toRange.upperBound)
        return xmlSeq.trimmingCharacters(in: .whitespacesAndNewlines)
    }
}


extension XmlRpcValue {

    func fromXML(xml: inout String.SubSequence) -> Bool {
        invalidate()

        var xmlSeq = xml

        if !XmlRpcUtil.nextTagIs(tag: .VALUE_TAG, xml: &xmlSeq) {
            return false
        }


        let tagString = XmlRpcUtil.getNextTag(xml: &xmlSeq)
        guard let tag = Tags(rawValue: tagString) else {
            return false
        }

        let valueString = xmlSeq.prefix(while: { $0 != "<" }).trimmingCharacters(in: .whitespacesAndNewlines)

        var result = false

        switch tag {
        case .BOOLEAN_TAG:
            if let b = Bool(valueString) {
                value = .boolean(b)
                result = true
            } else if let i = Int(valueString) {
                value = .boolean(i != 0)
                result = true
            }
        case .I4_TAG, .INT_TAG:
            if let i = Int(valueString) {
                value = .int(i)
                result = true
            }
        case .DOUBLE_TAG:
            if let d = Double(valueString) {
                value = .double(d)
                result = true
            }

        // Watch for empty/blank strings with no <string>tag
        case .STRING_TAG:
            value = .string(valueString)
            result = true
        case .DATETIME_TAG:
            result = timeFrom(xml: valueString)
        case .BASE64_TAG:
            result = false
        case .ARRAY_TAG:
            result = arrayFrom(xml: &xmlSeq)
        case .STRUCT_TAG:
            result = structFrom(xml: &xmlSeq)
        case .VALUE_ETAG:
            let str = XmlRpcUtil.parseTag(from: .VALUE_TAG, to: .VALUE_ETAG, xml: &xml)
            value = .string(str)
            return true
        default:
            result = false
        }

        if result {
            _ = XmlRpcUtil.findTag(tag: .VALUE_ETAG, xml: &xmlSeq)
            xml = xmlSeq
        }

        return result
    }

    func arrayFrom(xml: inout String.SubSequence) -> Bool {
        if !XmlRpcUtil.nextTagIs(tag: .DATA_TAG, xml: &xml) {
            return false
        }

        var array = [XmlRpcValue]()
        var v = XmlRpcValue()
        while v.fromXML(xml: &xml) {
            var newValue = XmlRpcValue()
            swap(&newValue, &v)
            array.append(newValue)
        }
        _ = XmlRpcUtil.nextTagIs(tag: .DATA_ETAG, xml: &xml)
        value = .array(array)
        return true
    }

    func structFrom(xml: inout String.SubSequence) -> Bool {
        var val = ValueStruct()
        while XmlRpcUtil.nextTagIs(tag: .MEMBER_TAG, xml: &xml) {
            let name = XmlRpcUtil.parseTag(from: .NAME_TAG, to: .NAME_ETAG, xml: &xml)
            let v = XmlRpcValue()
            _ = v.fromXML(xml: &xml)
            if !v.valid() {
                invalidate()
                return false
            }
            val[name] = v
            _ = XmlRpcUtil.nextTagIs(tag: .MEMBER_ETAG, xml: &xml)
        }
        value = .struct(val)
        return true
    }

    func timeFrom(xml: String) -> Bool {
        invalidate()
        guard xml.count == 17 else {
            return false
        }
        guard let year = Int(xml.prefix(4)),
            let mon = Int(xml.dropFirst(4).prefix(2)),
            let mday = Int(xml.dropFirst(6).prefix(2)),
            let hour = Int(xml.dropFirst(9).prefix(2)),
            let min = Int(xml.dropFirst(12).prefix(2)),
            let sec = Int(xml.suffix(2)) else {
                return false
        }
        var dateComponents = DateComponents()
        dateComponents.year = year
        dateComponents.month = mon
        dateComponents.day = mday
        dateComponents.timeZone = TimeZone(abbreviation: "GMT")
        dateComponents.hour = hour
        dateComponents.minute = min
        dateComponents.second = sec
        let userCalendar = Calendar.current // user calendar
        guard let date = userCalendar.date(from: dateComponents) else {
            return false
        }
        value = .datetime(date)
        return true
    }

}
