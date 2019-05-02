import Foundation
import StdMsgs
import RosTime


extension dynamic_reconfigure {

public struct ConfigDescription: Message {
public static var md5sum: String = "757ce9d44ba8ddd801bb30bc456f946f"
public static var datatype = "dynamic_reconfigure/ConfigDescription"
public static var definition = """
Group[] groups
Config max
Config min
Config dflt
"""
public static var hasHeader = false

public var groups: [Group]
public var max: Config
public var min: Config
public var dflt: Config

public init(groups: [Group], max: Config, min: Config, dflt: Config) {
self.groups = groups
self.max = max
self.min = min
self.dflt = dflt
}

public init() {
    groups = [Group]()
max = Config()
min = Config()
dflt = Config()
}

}
}
