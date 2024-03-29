import Foundation

let swiftType = ["int8": "Int8", "int16": "Int16", "int32": "Int32",
                 "int64": "Int64", "uint8": "UInt8", "uint16": "UInt16",
                 "uint32": "UInt32", "uint64": "UInt64", "string": "String",
                 "byte": "Int8", "char": "UInt8", "duration": "RosDuration",
                 "time": "Time", "bool": "Bool", "float32": "Float32",
                 "float64": "Float64", "empty": "Empty", "Header": "std_msgs.Header"]

extension MsgSpec {

    func generateStdMsgSwiftCode(context: MsgContext) -> String? {

        let data = text.trimmingCharacters(in: .whitespacesAndNewlines)

        let parts = data.components(separatedBy: .whitespaces)
            .map { $0.trimmingCharacters(in: .whitespacesAndNewlines )}
        let structName = parts[0] == "" ? "empty" : parts[0]
        let type = swiftType[structName]!
        let name = parts.count > 1 ? String(parts[1]) : "data"
        let md5sum = String(data).hashed() ?? "*"

        print("type = \(type)")
        var importStatement = ""
        if ["Time","Duration"].contains(type) {
            importStatement = "import RosTime"
        }

        let code = """
            // Generated by msgbuilder \(Date())

            \(importStatement)

            extension std_msgs {
            \tpublic struct \(structName): Message {
            \t\tpublic static let md5sum: String = "\(md5sum)"
            \t\tpublic static let datatype = "\(full_name)"
            \t\tpublic static let definition = "\(data)"

            \t\tpublic var \(name): \(type)

            \t\tpublic init(_ value: \(type)) {
            \t\t\tself.\(name) = value
            \t\t}
            \t}
            }
            """

        return code
    }
}
