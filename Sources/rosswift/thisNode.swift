//
//  thisNode.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

import Foundation
import RosTime

extension Ros {
    final class ThisNode {
        static let instance = ThisNode()
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

        class func initialize(name: String, remappings: StringStringMap, options: InitOption) {
            instance.initialize(name: name, remappings: remappings, options: options)
        }

        private func initialize(name inName: String, remappings: StringStringMap, options: InitOption) {
            if let namespaceEnvironment = ProcessInfo.processInfo.environment["ROS_NAMESPACE"] {
                namespace = namespaceEnvironment
            }

            guard !name.isEmpty else {
                fatalError("The node name must not be empty")
            }

            name = inName

            var disableAnon = false
            if let it = remappings["__name"] {
                self.name = it
                disableAnon = true
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
                fatalError("Namespace [\(namespace)] is invalid: \(error)")
            }

            // names must be initialized here, because it requires the namespace
            // to already be known so that it can properly resolve names.
            // It must be done before we resolve g_name, because otherwise the name will not get remapped.
            Names.initialize(remappings: remappings)

            if name.contains("/") {
                fatalError("\(name), node names cannot contain /")
            }

            if name.contains("~") {
                fatalError("\(name), node names cannot contain ~")
            }

            name = Names.resolve(ns: namespace, name: name)

            if options.contains(.anonymousName) && !disableAnon {
                name.append("_\(RosTime.WallTime.now().toNSec())")
            }

            Ros.Console.setFixedFilterToken(key: "node", val: name)
        }

    }
}
