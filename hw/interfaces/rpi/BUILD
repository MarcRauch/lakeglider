load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "spiRpi",
    srcs = ["SpiRpi.cc"],
    hdrs = [
        "SpiRpi.hh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//hw:pins",
        "//hw/interfaces:spiInterface",
    ],
)
