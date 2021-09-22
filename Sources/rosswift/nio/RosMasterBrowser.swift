//
//  RosMasterBrowser.swift
//  
//
//  Created by Thomas Gustafsson on 2021-02-22.
//

import Foundation
#if os(Linux)
import NetService
#endif

#if os(macOS) || os(iOS) || os(tvOS) || os(Linux)
final class RosMasterBrowser: NSObject, NetServiceBrowserDelegate, NetServiceDelegate  {
    let browser = NetServiceBrowser()
    var host = ""
    var port = 11311
    
    override init() {
        super.init()
        browser.delegate = self
    }

    func start() {
        browser.searchForServices(ofType: "_ros._tcp.", inDomain: "local.")
    }
    
    //MARK: NetServiceBrowserDelegate
    
    func netServiceBrowser(_ browser: NetServiceBrowser, didFind service: NetService, moreComing: Bool) {
        service.delegate = self
        service.resolve(withTimeout: 10)
        RunLoop.current.run(until: .init(timeIntervalSinceNow: 1))
    }
    
    func netServiceBrowser(_ browser: NetServiceBrowser, didNotSearch errorDict: [String : NSNumber]) {
        print(errorDict)
        host = "error"
    }
    
    //MARK: NetServiceDelegate
    
    func netServiceDidResolveAddress(_ sender: NetService) {
        if let hostname = sender.hostName {
            ROS_DEBUG("rosmaster found: http://\(hostname):\(sender.port)")
            host = hostname
            port = sender.port
        }
    }
    
}
#endif
