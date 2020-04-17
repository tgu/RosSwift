private let PRIV_NAME = "~"


func isPrivate(_ name: String) -> Bool {
    return name.hasPrefix(PRIV_NAME)
}

func isGlobal(_ name: String) -> Bool {
    return name.hasPrefix("/")
}

/// Get the namespace of name. The namespace is returned with a
/// trailing slash in order to favor easy concatenation and easier use
/// within the global context.

func namespace(name: String) -> String {
    if name.isEmpty || name == "/" {
        return "/"
    }

    var  strippedName = name
    if name.hasSuffix("/") {
        strippedName = String(name.dropLast())
    }
    if !strippedName.hasPrefix("/") {
        strippedName = "/" + strippedName
    }

    if let lastPos = strippedName.lastIndex(of: "/") {
        if lastPos == strippedName.startIndex {
            return "/"
        }
        return String(strippedName.prefix(upTo: lastPos)) + "/"
    }

    return "/"
}

/// Put name in canonical form. Extra slashes '//' are removed and
/// name is returned without any trailing slash, e.g. /foo/bar

func canonicalize(name: String) -> String {
    if name.isEmpty || name == "/" {
        return name
    }

    if name.hasPrefix("/") {
        return "/" + name.split(separator: "/").joined(separator: "/")
    } else {
        return name.split(separator: "/").joined(separator: "/")
    }

}

/// Resolve a ROS name to its global, canonical form. Private ~names
/// are resolved relative to the node name.

func resolve(name: String, nameSpace: String, remappings: [String: String]? = nil) -> String {
    if name.isEmpty {
        // empty string resolves to parent of the namespace_
        return namespace(name: nameSpace)
    }

    var resolvedName = canonicalize(name: name)
    if resolvedName.hasPrefix("/") {

    } else if isPrivate(name) {
        resolvedName = canonicalize(name: nameSpace + "/" + name.dropFirst())
    } else {
        resolvedName = namespace(name: nameSpace) + name
    }

    return remappings?[resolvedName] ?? resolvedName
}


/// Join a namespace and name. If name is unjoinable (i.e. ~private or
/// /global) it will be returned without joining

func join(namespace: String, name: String) -> String {
    if isPrivate(name) || isGlobal(name) {
        return name
    }

    if namespace == PRIV_NAME {
        return PRIV_NAME + name
    }

    if namespace.isEmpty {
        return name
    }

    if namespace.hasSuffix("/") {
        return namespace + name
    }

    return namespace + "/" + name
}
