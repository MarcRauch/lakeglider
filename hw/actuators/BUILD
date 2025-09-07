load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "actuators",
    srcs = [
        "Pump.cc",
    ],
    hdrs = [
        "Pump.hh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//com/msg:msgs",
        "//hw:pins",
        "//hw/interfaces:adcInterface",
        "//hw/interfaces:gpioInterface",
        "//hw/interfaces:uartInterface",
    ],
)
