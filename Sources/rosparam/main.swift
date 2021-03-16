import RosSwift

let ros = Ros(name: "rosparam", options: [.anonymousName])

if let result = try? ros.param.getParameterNames().wait() {
    print(result.map { $0 }.joined(separator: "\n"))
} else {
    print("call failed")
}


