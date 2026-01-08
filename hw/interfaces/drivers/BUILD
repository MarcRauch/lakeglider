load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "canInterface",
    srcs = [
        "CanManager.cc",
        "Mcp2515.cc",
    ],
    hdrs = [
        "CanManager.hh",
        "Mcp2515.hh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//hw:pins",
        "//hw/interfaces:spiInterface",
        "//utils/time:timeInterface",
    ],
)
