import rpcclient
import rpcobject
import NIO
import RosNetwork

let threadGroup = MultiThreadedEventLoopGroup(numberOfThreads: 1)

let host = RosNetwork.determineHost()
let port = 11311

let msg = XmlRpcValue(str: "/rosparam-\(getpid())")

let master = nio.Master(group: threadGroup)


guard let client = try? master.connect(host: host, port: Int(port)).wait() else {
        print("Could not connect to master")
        exit(1)
}

print("connected to master")

guard let result = try? client.send(method: "getParamNames", request: msg).wait() else {
    print("no response")
    exit(1)
}

print("result \(result)")

switch result {
case .success(let response):
    for parameter in response[2] {
        print(parameter)
    }
case .failure(let error):
    print("Error in getParamNames: \(error.localizedDescription)")
}
