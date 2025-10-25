load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "timeInterface",
    srcs = [],
    hdrs = [
        "Time.hh",
        "IClock.hh",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "timePico",
    srcs = ["PicoClock.cc"],
    hdrs = ["PicoClock.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":timeInterface",
        "@pico-sdk//src/rp2_common/pico_stdlib:pico_stdlib",
    ],
)

cc_library(
    name = "timeLinux",
    srcs = ["LinuxClock.cc"],
    hdrs = ["LinuxClock.hh"],
    visibility = ["//visibility:public"],
    deps = [":timeInterface"],
)
