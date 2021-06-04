//
//  thisNode.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-03-03.
//

extension Ros {

    /// Get the list of topics advertised by this node
    ///
    public func getAdvertisedTopics() -> [String] {
        return topicManager.getAdvertised()
    }

    /// Get the list of topics subscribed to by this node

    public func getSubscribedTopics() -> [String] {
        return topicManager.getSubscribed()
    }

}
