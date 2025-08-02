load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "adcMock",
    srcs = ["AdcMock.cc"],
    hdrs = [
        "AdcMock.hh",
    ],
    visibility = ["//visibility:public"],
    deps = ["//hw/interfaces:adcInterface"],
)

cc_library(
    name = "i2cMock",
    srcs = ["I2cMock.cc"],
    hdrs = [
        "I2cMock.hh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//hw/interfaces:i2cInterface",
    ],
)
