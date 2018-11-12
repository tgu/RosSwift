//
//  Service.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-11.
//

import Foundation
import StdMsgs
import RosTime
import NIO



public struct service {

    public static func call<MReq: ServiceMessage, MRes: ServiceMessage>(service_name: String, req: MReq) -> EventLoopFuture<MRes?> {
        let nh = Ros.NodeHandle()
        let client = nh.serviceClient(service: Ros.Names.resolve(name: service_name), md5sum: MReq.srv_md5sum)
        return client.call(req: req)
    }

    static func call<Service: ServiceProt>(name: String, service: inout Service) -> Bool {
        return call(service_name: name, req: service.request, response: &service.response)
    }

    static func call<MReq: ServiceMessage, MRes: ServiceMessage>(service_name: String, req: MReq, response: inout MRes) -> Bool {
        let res : EventLoopFuture<MRes?> = call(service_name: service_name, req: req)
        do {
            if let resp = try res.wait() {
                response = resp
                return true
            }
        } catch {
            return false
        }
        return false

    }


    /// Wait for a service to be advertised and available.  Blocks until it is.
    ///
    /// - Parameters:
    ///   - service_name: Name of the service to wait for
    ///   - timeout: The amount of time to wait for, in milliseconds.  If timeout is -1, waits until the node is shutdown
    /// - Returns: true on success, false otherwise
    static func waitForService(service_name: String, timeout: Int32) -> Bool {
        let dur = RosTime.Duration(milliseconds: timeout)
        return waitForService(service_name: service_name, timeout: dur )
    }

    /**
     * \brief Wait for a service to be advertised and available.  Blocks until it is.
     * \param service_name Name of the service to wait for.
     * \param timeout The amount of time to wait for before timing out.  If timeout is -1 (default),
     * waits until the node is shutdown
     * \return true on success, false otherwise
     */
    static func waitForService(service_name: String, timeout: RosTime.Duration = RosTime.Duration(seconds: TimeInterval(-1))) -> Bool {
        let mapped_names = Ros.Names.resolve(name: service_name)
        let start_time = RosTime.Time.now()
        var printed = false
        var result = false
        while Ros.ok {
            if exists(service_name: service_name, print_failure_reason: !printed) {
                result = true
                break
            } else {
                printed = true
                if timeout >= RosTime.Duration(seconds: 0) {
                    let current_time = RosTime.Time.now()
                    if current_time - start_time >= timeout {
                        return false
                    }
                }

                RosTime.Duration(milliseconds: 20).sleep()
            }
        }

        if printed && Ros.ok {
            ROS_DEBUG("waitForService: Service [\(mapped_names)] is now available.")
        }

        return result
    }


    /**
     * \brief Checks if a service is both advertised and available.
     * \param service_name Name of the service to check for
     * \param print_failure_reason Whether to print the reason for failure to the console (service not advertised vs.
     * could not connect to the advertised host)
     * \return true if the service is up and available, false otherwise
     */

    static func callback(m: M_string) {
        ROS_DEBUG(m.debugDescription)
    }


    static func exists(service_name : String, print_failure_reason: Bool) -> Bool {
        let mapped_name = Ros.Names.resolve(name: service_name)
        var host = ""
        var port : UInt16 = 0
        if ServiceManager.instance.lookupService(name: mapped_name, serv_host: &host, serv_port: &port) {
            let keymap = ["probe":"1","md5sum":"*","callerid":Ros.this_node.getName(),"service":mapped_name]
            let transport = nio.TransportTCP(pipeline: [nio.MessageDelimiterCodec(),nio.HeaderMessageCodec(),nio.TransportTCP.Handler(callback: callback)])
            do {
                
            try transport.connect(host: host, port: Int(port)).map { channel -> Void in
                channel.writeAndFlush(keymap)
            }.wait()
            return true
            } catch {
                ROS_ERROR("\(error)")
                return false
            }

        } else {
            if print_failure_reason {
                ROS_DEBUG("waitForService: Service [\(mapped_name)] has not been advertised, waiting...")
            }
        }
        return false
    }

}

