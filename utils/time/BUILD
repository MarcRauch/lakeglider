load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "timeInterface",
    srcs = [],
    hdrs = [
        "GlTime.hh",
        "ITimeProvider.hh",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "timePico",
    srcs = ["PicoTimeProvider.cc"],
    hdrs = ["PicoTimeProvider.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":timeInterface",
        "@pico-sdk//src/rp2_common/pico_stdlib:pico_stdlib",
    ],
)

cc_library(
    name = "timeLinux",
    srcs = ["LinuxTimeProvider.cc"],
    hdrs = ["LinuxTimeProvider.hh"],
    visibility = ["//visibility:public"],
    deps = [":timeInterface"],
)
