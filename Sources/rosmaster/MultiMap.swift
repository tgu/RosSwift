//
//  Multimap.swift
//  Buckets
//
//  Created by Mauricio Santos on 4/1/15.
//  Copyright (c) 2015 Mauricio Santos. All rights reserved.
//

import Foundation

/// A Multimap is a dictionary in which each key may be
/// associated with multiple values. This implementation allows duplicate key-value pairs.
///
/// Comforms to `Sequence`, `ExpressibleByDictionaryLiteral`,
/// `Equatable` and `CustomStringConvertible`
public struct Multimap<Key: Hashable, Value: Equatable> {
    fileprivate var storage = [Key: [Value]]()

    init() {

    }

    /// Number of keys
    public var keyCount: Int {
        return storage.count
    }

    public var isEmpty: Bool {
        return storage.isEmpty
    }

    /// A sequence containing the keys.
    public var keys: AnySequence<Key> {
        return AnySequence(storage.keys)
    }

    /// A sequence containing the values.
    public var values: AnySequence<Value> {
        let selfIterator = makeIterator()
        let valueIterator = AnyIterator { selfIterator.next()?.1 }
        return AnySequence(valueIterator)
    }

    /// Returns the values associated with the specified key.
    /// An empty array is returned if the key does not exist.
    public subscript(key: Key) -> [Value] {
        return storage[key] ?? []
    }

    /// Returns `true` if the multimap contains at least one key-value pair with the given key.
    public func contains(key: Key) -> Bool {
        return storage[key] != nil
    }

    /// Returns `true` if the multimap contains at least one key-value pair with the given key and value.
    public func contains(value: Value, forKey key: Key) -> Bool {
        return storage[key]?.contains(value) ?? false
    }

    /// Inserts a key-value pair into the multimap.
    public mutating func insert(value: Value, forKey key: Key) {
        if storage.keys.contains(key) {
            storage[key]!.append(value)
        } else {
            storage[key] = [value]
        }
    }

    /// Removes a single key-value pair with the given key and value from the multimap, if it exists.
    ///
    /// - returns: The removed value, or nil if no matching pair is found.
    @discardableResult
    public mutating func removeValue(_ value: Value, forKey key: Key) -> Value? {
        if var values = storage[key] {
            if let removeIndex = values.firstIndex(of: value) {
                let removedValue = values.remove(at: removeIndex)
                if values.count > 0 {
                    storage[key] = values
                } else {
                    storage.removeValue(forKey: key)
                }
                return removedValue
            }
            if values.isEmpty {
                storage.removeValue(forKey: key)
            }
        }
        return nil
    }

    /// Removes all values associated with the given key.
    public mutating func removeValuesForKey(_ key: Key) {
        storage.removeValue(forKey: key)
    }

    /// Removes all the elements from the multimap, and by default
    /// clears the underlying storage buffer.
    public mutating func removeAll(keepingCapacity keep: Bool = true) {
        storage.removeAll(keepingCapacity: keep)
    }

}

extension Multimap: Sequence {
    public func makeIterator() -> AnyIterator<(Key,Value)> {
        var keyValueGenerator = storage.makeIterator()
        var index = 0
        var pairs = keyValueGenerator.next()
        return AnyIterator {
            if let (key,values) = pairs {
                let value = values[index]
                index += 1
                if index >= values.count {
                    index = 0
                    pairs = keyValueGenerator.next()
                }
                return (key, value)
            }
            return nil
        }
    }
}

extension Multimap: CustomStringConvertible {
    public var description: String {
        return "[" + map{"\($0.0): \($0.1)"}.joined(separator: ", ") + "]"
    }
}
