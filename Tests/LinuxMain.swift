import XCTest

import msgBuilderTests
import rosmasterTests
import rosswiftTests

var tests = [XCTestCaseEntry]()
tests += msgBuilderTests.__allTests()
tests += rosmasterTests.__allTests()
tests += rosswiftTests.__allTests()

XCTMain(tests)
