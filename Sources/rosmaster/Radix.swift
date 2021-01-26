// Copyright (c) 2016 Matthijs Hollemans and contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.import Foundation

// Code adapted from: https://github.com/raywenderlich/swift-algorithm-club.git

import Logging
import rpcobject

protocol ArrayConstructable: Collection {
    init()
    init(any: Any)
    var count: Int { get }
    var dictionary: [String: Self]? { get }
}

extension XmlRpcValue: ArrayConstructable {}

fileprivate let logger = Logger(label: "radix")

// The root is the top of the Radix Tree

class Root<T: ArrayConstructable> {
    var children: [Edge<T>]

    init() {
        children = [Edge<T>]()
    }

    deinit {
        logger.debug(" Root with \(children.count) removed")
    }

    // Returns the length (in number of edges) of the longest traversal down the tree.
    var height: Int {
        // Base case: no children: the tree has a height of 1
        if children.count == 0 {
            return 1
        }
            // Recursion: find the max height of a root's child and return 1 + that max
        else {
            var max = 1
            for c in children {
                if c.height > max {
                    max = c.height
                }
            }
            return 1 + max
        }
    }

    func getNames(_ names: inout [String]) {
        for c in children {
            c.getNames(&names)
        }
    }

    var values: T {
        var dictionary = [String: T]()
        for c in children {
            dictionary[c.label] = c.values
        }
        return T(any: dictionary)
    }

    var fullName: String {
        return ""
    }


    // Returns how far down in the tree a Root/Edge is.
    // A root's level is always 0.
    public var level: Int {
        return 0
    }

    // Prints the tree for debugging/visualization purposes.
    public func printRoot() {
        // Print the first half of the children
        if children.count > 1 {
            for c in 0...children.count/2-1 {
                children[c].printEdge()
                print("|")
            }
        } else if children.count > 0 {
            children[0].printEdge()
        }
        // Then print the root
        print("ROOT")
        // Print the second half of the children
        if children.count > 1 {
            for c in children.count/2...children.count-1 {
                children[c].printEdge()
                print("|")
            }
        }
        print()
    }
}

// Edges are what actually store the Strings in the tree
final class Edge<T: ArrayConstructable>: Root<T> {
    var parent: Root<T>
    var label: String
    var value: T

    init(_ label: String, parent: Root<T>, value: T) {
        let ns = label.split(separator: "/")
        self.label = String(ns[0])
        self.value = value
        self.parent = parent
        super.init()
        if ns.count > 1 {
            let edge = Edge(ns.dropFirst().joined(separator: "/"), parent: self, value: value)
            children.append(edge)
            self.value = T()
        }
    }

    override var fullName: String {
        return parent.fullName + "/" + label
    }

    override func getNames(_ names: inout [String]) {
        if children.count > 0 {
            super.getNames(&names)
        } else {
            names.append(fullName)
        }
    }

    override var values: T {
        if children.count > 0 {
            return super.values
        }
        return value
    }

    override var level: Int {
        return 1 + parent.level
    }

    // Prints the tree for debugging/visualization purposes.
    func printEdge() {
        // Print the first half of the edge's children
        if children.count > 1 {
            for c in 0...children.count/2-1 {
                children[c].printEdge()
            }
        } else if children.count > 0 {
            children[0].printEdge()
        }
        // Tab over once up to the edge's level
        for x in 1...level {
            if x == level {
                print("|------>", terminator: "")
            } else {
                print("|       ", terminator: "")
            }
        }
        print("\(label):\(value)")
        // Print the second half of the edge's children
        if children.count == 0 {
            for _ in 1...level {
                print("|       ", terminator: "")
            }
            print()
        }
        if children.count > 1 {
            for c in children.count/2...children.count-1 {
                children[c].printEdge()
            }
        }
    }
}

final class RadixTree<T: ArrayConstructable> {
    typealias EdgeType = Edge<T>
    let root: Root<T>

    init() {
        root = Root()
    }

    // Returns the height of the tree by calling the root's height function.
    var height: Int {
        return root.height - 1
    }

    // Inserts a parameter into the tree.
    @discardableResult
    func insert(_ str: String, value: T) -> Edge<T>? {
        //Account for a blank input. The empty string is already in the tree.
        if str.isEmpty {
            return nil
        }

        // searchStr is the parameter of the function
        // it will be substringed as the function traverses down the tree
        var searchStr = str.hasPrefix("/") ? String(str.dropFirst()) : str

        // currEdge is the current Edge (or Root) in question
        var currEdge = root

        while true {
            var found = false

            // If the current Edge has no children then the remaining searchStr is
            // created as a child
            if currEdge.children.count == 0 {
                let newEdge = EdgeType(searchStr, parent: currEdge, value: value)
                currEdge.children.append(newEdge)
                return newEdge
            }

            // Loop through all of the children
            for e in currEdge.children {
                // Get the shared prefix between the child in question and the
                // search string
                let shared = sharedPrefix(searchStr, e.label)

                // If the search string is equal to the shared string,
                // the string already exists in the tree
                if searchStr == shared {
                    return nil
                }

                    // If the child's label is equal to the shared string, you have to
                    // traverse another level down the tree, so substring the search
                    // string, break the loop, and run it back
                else if shared == e.label {
                    currEdge = e
                    searchStr = String(searchStr.dropFirst(shared.count+1))
                    found = true
                    break
                }
                // If they don't share a prefix, go to next child
            }

            // If none of the children share a prefix, you have to create a new child
            if !found {
                let newEdge = EdgeType(searchStr, parent: currEdge, value: value)
                currEdge.children.append(newEdge)
                return newEdge
            }
        }
    }

    func find(_ str: String) -> Bool {
        return get(str) != nil
    }

    func get(_ str: String) -> Root<T>? {
        if str == "/" || str.isEmpty {
            return root
        }

        var currEdge = root
        for p in str.split(separator: "/") {
            guard let edge = currEdge.children.first(where: { (edge) -> Bool in edge.label == p}) else {
                return nil
            }
            currEdge = edge
        }
        return currEdge
    }

    // Removes a parameter branch from the tree
    public func remove(_ str: String) -> Edge<T>? {

        if let edge = get(str) as? EdgeType {
            let parent = edge.parent
            let index = parent.children.firstIndex { $0.label == edge.label }
            parent.children.remove(at: index!)
            if parent.children.count == 0, let par = parent as? EdgeType {
                _ = remove(par.fullName)
            }
            return edge
        }

        return nil
    }

    // Prints the tree by calling the root's print function
    public func printTree() {
        root.printRoot()
    }

    func getNames() -> [String] {
        var names = [String]()
        root.getNames(&names)
        return names
    }
}

// Returns the prefix that is shared between the two input strings
// i.e. sharedPrefix("co/urt", "co/ral") -> "co"
fileprivate func sharedPrefix(_ str1: String, _ str2: String) -> String {
    precondition(!str1.hasPrefix("/") && !str2.hasPrefix("/"))
    let p1 = str1.split(separator: "/")
    let p2 = str2.split(separator: "/")
    
    return zip(p1, p2)
        .prefix(while: { $0 == $1 })
        .map { $0.0 }
        .joined(separator: "/")
}
