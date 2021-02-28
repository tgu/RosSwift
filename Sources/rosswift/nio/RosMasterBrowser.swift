//
//  File.swift
//  
//
//  Created by Thomas Gustafsson on 2021-02-22.
//

import Foundation
#if os(Linux)
import NetService
#endif

final class RosMasterBrowser: NSObject, NetServiceBrowserDelegate, NetServiceDelegate  {
    let browser = NetServiceBrowser()
    var host = ""
    var port = 11311
    
    override init() {
        super.init()
        browser.delegate = self
    }

    func start() {
        ROS_DEBUG("Search for ros master domains")
        browser.searchForServices(ofType: "_ros._tcp.", inDomain: "")
    }
    
    //MARK: NetServiceBrowserDelegate
    
    func netServiceBrowser(_ browser: NetServiceBrowser, didFind service: NetService, moreComing: Bool) {
        service.delegate = self
        print("starting resolve")
        service.resolve(withTimeout: 10)
        print("returned resolve")
        RunLoop.current.run(until: .init(timeIntervalSinceNow: 1))
        print("runloop finished")
    }
    
    //MARK: NetServiceDelegate
    
    func netServiceDidResolveAddress(_ sender: NetService) {
        if let hostname = sender.hostName {
            print("rosmaster found: http://\(hostname):\(sender.port)")
            host = hostname
            port = sender.port
        }
    }

}
