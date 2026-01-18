load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "msgs",
    srcs = ["Msg.cc"],
    hdrs = [
        "Battery.hh",
        "Compass.hh",
        "Depth.hh",
        "Header.hh",
        "Imu.hh",
        "Msg.hh",
        "Time.hh",
        "Type.hh",
    ],
    visibility = ["//visibility:public"],
    deps = ["//utils/time:timeInterface"],
)
