import XCTest
@testable import rosswiftTests

XCTMain([
    testCase(rosswiftTests.allTests),
    testCase(paramTests.allTests),
    testCase(serviceTests.allTests),
    testCase(connectionTests.allTests),
    testCase(XmlRpcValueTest.allTests),
    testCase(serializationTests.allTests),
    testCase(NameRemappingWithNamespace.allTests)
])
