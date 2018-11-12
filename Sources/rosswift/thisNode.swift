//
//  thisNode.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation
import RosTime

extension Ros {
    final class this_node {
        static let instance = this_node()
        var name = "empty"
        public var namespace = "/"

        private init() {
        }

        class func getName() -> String {
            return instance.name
        }

        class func getNamespace() -> String {
            return instance.namespace
        }

        class func getAdvertisedTopics(topics: inout [String]) {
            TopicManager.instance.getAdvertised(topics: &topics)
        }

        class func getSubscribedTopics(topics: inout [String]) {
            TopicManager.instance.getSubscribed(topics: &topics)
        }


        class func initialize(name: String, remappings: M_string, options: InitOption)
        {
            instance.initialize(name: name, remappings: remappings, options: options)
        }

        private func initialize(name in_name: String, remappings: M_string, options: InitOption) {
            if let ns_env = ProcessInfo.processInfo.environment["ROS_NAMESPACE"] {
                namespace = ns_env
            }

            guard !name.isEmpty else {
                fatalError("The node name must not be empty")
            }

            name = in_name

            var disable_anon = false
            if let it = remappings["__name"] {
                self.name = it
                disable_anon = true
            }

            if let it = remappings["__ns"] {
                self.namespace = it
            }

            namespace = Names.clean(namespace)
            if namespace.isEmpty || namespace.first != "/" {
                namespace = "/" + namespace
            }

            var error = ""
            if !Names.validate(name: namespace, error: &error) {
                let ss = "Namespace [\(namespace)] is invalid: \(error)"
                fatalError(ss)
            }

            // names must be initialized here, because it requires the namespace to already be known so that it can properly resolve names.
            // It must be done before we resolve g_name, because otherwise the name will not get remapped.
            Names.initialize(remappings: remappings)

            if name.contains("/") {
                fatalError("\(name), node names cannot contain /")
            }

            if name.contains("~") {
                fatalError("\(name), node names cannot contain ~")
            }

            name = Names.resolve(ns: namespace, name: name)

            if options.contains(.AnonymousName) && !disable_anon {
                name.append("_\(RosTime.WallTime.now().toNSec())")
            }

            Ros.console.setFixedFilterToken(key: "node", val: name)
        }

    }
}
