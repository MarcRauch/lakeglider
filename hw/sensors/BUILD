load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "sensors",
    srcs = [
        "Battery.cc",
        "Ms5837.cc",
    ],
    hdrs = [
        "Battery.hh",
        "Ms5837.hh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//com/msg:msgs",
        "//hw:pins",
        "//hw/interfaces:adcInterface",
        "//hw/interfaces:i2cInterface",
        "//utils/time:timeInterface"
    ],
)
