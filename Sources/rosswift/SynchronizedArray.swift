//
//  SynchronizedArray.swift
//  RosSwift
//
//  Created by Thomas Gustafsson on 2018-10-29.
//

import Foundation
import Synchronization

/// A thread-safe array.
final class SynchronizedArray<Element>: Sendable {
    fileprivate let array = Mutex<[Element]>([])
}

// MARK: - Properties
extension SynchronizedArray {
    
    /// The first element of the collection.
    var first: Element? {
        array.withLock { $0.first }
    }
    
    /// The last element of the collection.
    var last: Element? {
        array.withLock { $0.last }
    }
    
    /// The number of elements in the array.
    var count: Int {
        array.withLock { $0.count }
    }
    
    /// A Boolean value indicating whether the collection is empty.
    var isEmpty: Bool {
        array.withLock { $0.isEmpty }
    }
    
    /// A textual representation of the array and its elements.
    var description: String {
        array.withLock { $0.description }
    }
    
    func all() -> [Element] {
        array.withLock  { $0 }
    }
    
    /// Removes all elements from the array.
    ///
    /// - Parameter completion: The handler with the removed elements.
    func removeAll(completion: (([Element]) -> Void)? = nil) {
        let elements = array.withLock {
            let elements = $0
            $0.removeAll()
            return elements
        }
        if let completion {
            completion(elements)
        }
    }
    
    /// Returns a Boolean value indicating whether the sequence contains an element that satisfies the given predicate.
    ///
    /// - Parameter predicate: A closure that takes an element of the sequence as its argument and returns a Boolean value that indicates whether the passed element represents a match.
    /// - Returns: true if the sequence contains an element that satisfies predicate; otherwise, false.
    func firstIndex(where predicate: @Sendable (Element) -> Bool) -> Int? {
        array.withLock { $0.firstIndex(where: predicate) }
    }
    
    
}

// MARK: - Immutable
extension SynchronizedArray where Element: Sendable {
    
    func removeAll(where predicate: (Element) -> Bool) {
        array.withLock { $0.removeAll(where: predicate) }
    }
    
    
    
    /// Returns the first element of the sequence that satisfies the given predicate or nil if no such element is found.
    ///
    /// - Parameter predicate: A closure that takes an element of the sequence as its argument and returns a Boolean value indicating whether the element is a match.
    /// - Returns: The first match or nil if there was no match.
    func first(where predicate: (Element) -> Bool) -> Element? {
        array.withLock { $0.first(where: predicate) }
    }
    
    /// Returns an array containing, in order, the elements of the sequence that satisfy the given predicate.
    ///
    /// - Parameter isIncluded: A closure that takes an element of the sequence as its argument and returns a Boolean value indicating whether the element should be included in the returned array.
    /// - Returns: An array of the elements that includeElement allowed.
    func filter(_ isIncluded: (Element) -> Bool) -> [Element] {
        array.withLock { $0.filter(isIncluded) }
    }
    
    func filterSelf(_ isIncluded: (Element) -> Bool) {
        array.withLock { $0 = $0.filter(isIncluded) }
    }
    
    
    
    /// Returns the first index in which an element of the collection satisfies the given predicate.
    ///
    /// - Parameter predicate: A closure that takes an element as its argument and returns a Boolean value that indicates whether the passed element represents a match.
    /// - Returns: The index of the first element for which predicate returns true. If no elements in the collection satisfy the given predicate, returns nil.
    func index(where predicate: (Element) -> Bool) -> Int? {
        array.withLock { $0.firstIndex(where: predicate) }
    }
    
    /// Returns the elements of the collection, sorted using the given predicate as the comparison between elements.
    ///
    /// - Parameter areInIncreasingOrder: A predicate that returns true if its first argument should be ordered before its second argument; otherwise, false.
    /// - Returns: A sorted array of the collection’s elements.
    func sorted(by areInIncreasingOrder: (Element, Element) -> Bool) -> [Element] {
        array.withLock { $0.sorted(by: areInIncreasingOrder) }
    }
    
    /// Returns an array containing the non-nil results of calling the given transformation with each element of this sequence.
    ///
    /// - Parameter transform: A closure that accepts an element of this sequence as its argument and returns an optional value.
    /// - Returns: An array of the non-nil results of calling transform with each element of the sequence.
    func compactMap<ElementOfResult: Sendable>(_ transform: (Element) -> ElementOfResult?) -> [ElementOfResult] {
        array.withLock { $0.compactMap(transform) }
    }
    
    /// Calls the given closure on each element in the sequence in the same order as a for-in loop.
    ///
    /// - Parameter body: A closure that takes an element of the sequence as a parameter.
    func forEach(_ body: (Element) -> Void) {
        array.withLock { $0.forEach(body) }
    }
    
    func map<T: Sendable>(_ body: (Element) -> T) -> [T] {
        array.withLock {
            $0.map(body)
        }
    }
    
    /// Returns a Boolean value indicating whether the sequence contains an element that satisfies the given predicate.
    ///
    /// - Parameter predicate: A closure that takes an element of the sequence as its argument and returns a Boolean value that indicates whether the passed element represents a match.
    /// - Returns: true if the sequence contains an element that satisfies predicate; otherwise, false.
    func contains(where predicate: (Element) -> Bool) -> Bool {
        array.withLock { $0.contains(where: predicate) }
    }
    
    
}

// MARK: - Mutable
extension SynchronizedArray where Element: Sendable {
    
    /// Adds a new element at the end of the array.
    ///
    /// - Parameter element: The element to append to the array.
    func append( _ element: Element) {
        array.withLock {
            $0.append(element)
        }
    }
    
    /// Adds a new element at the end of the array.
    ///
    /// - Parameter element: The element to append to the array.
    func append( _ elements: [Element]) {
        array.withLock {
            $0 += elements
        }
    }
    
    /// Inserts a new element at the specified position.
    ///
    /// - Parameters:
    ///   - element: The new element to insert into the array.
    ///   - index: The position at which to insert the new element.
    func insert( _ element: Element, at index: Int) {
        array.withLock {
            $0.insert(element, at: index)
        }
    }
    
    /// Removes and returns the element at the specified position.
    ///
    /// - Parameters:
    ///   - index: The position of the element to remove.
    ///   - completion: The handler with the removed element.
    
    func remove(at index: Int, completion: (@Sendable (Element) -> Void)) {
        let element = array.withLock {
            $0.remove(at: index)
        }
        
        // must do this to decrease the retain count of element
        completion(element)
        
    }
    
    
    
    func remove(at index: Int) {
        array.withLock {
            _ = $0.remove(at: index)
        }
    }
    
    
    /// Removes and returns the element at the specified position.
    ///
    /// - Parameters:
    ///   - predicate: A closure that takes an element of the sequence as its argument and returns a Boolean value indicating whether the element is a match.
    ///   - completion: The handler with the removed element.
    func remove(where predicate: @escaping (Element) -> Bool, completion: (@Sendable (Element) -> Void)? = nil) {
        array.withLock {
            guard let index = $0.firstIndex(where: predicate) else {
                return
            }
            let element = $0.remove(at: index)
            if completion != nil {
                Task {
                    completion?(element)
                }
            }
        }
    }
    
}

extension SynchronizedArray where Element: Sendable {
    
    /// Accesses the element at the specified position if it exists.
    ///
    /// - Parameter index: The position of the element to access.
    /// - Returns: optional element if it exists.
    subscript(index: Int) -> Element? {
        get {
            var result: Element?
            
            array.withLock {
                guard $0.startIndex..<$0.endIndex ~= index else {
                    return
                }
                result = $0[index]
            }
            
            return result
        }
        set {
            guard let newValue = newValue else {
                return
            }
            
            array.withLock {
                $0[index] = newValue
            }
        }
    }
}

// MARK: - Equatable
extension SynchronizedArray where Element: Equatable & Sendable {
    
    /// Returns a Boolean value indicating whether the sequence contains the given element.
    ///
    /// - Parameter element: The element to find in the sequence.
    /// - Returns: true if the element was found in the sequence; otherwise, false.
    func contains(_ element: Element) -> Bool {
        array.withLock { $0.contains(element) }
    }
}

// MARK: - Infix operators
extension SynchronizedArray where Element: Sendable {
    
    static func += (left: inout SynchronizedArray, right: Element) {
        left.append(right)
    }
    
    static func += (left: inout SynchronizedArray, right: [Element]) {
        left.append(right)
    }
}


extension SynchronizedArray where Element: Equatable & Sendable {
    func firstIndex(of element: Element) -> Int? {
        array.withLock { $0.firstIndex(of: element) }
    }
    
}
