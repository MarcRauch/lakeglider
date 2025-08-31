load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "battery_test",
    srcs = ["battery_test.cc"],
    deps = [
        "//hw/interfaces/mock:adcMock",
        "//hw/sensors",
        "@googletest//:gtest_main",
    ],
)
