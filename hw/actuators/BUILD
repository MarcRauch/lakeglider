load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "actuators",
    srcs = [
        "Pump.cc",
        "Tmc5160.cc",
    ],
    hdrs = [
        "Pump.hh",
        "Tmc5160.hh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//com/msg:msgs",
        "//hw:pins",
        "//hw/interfaces:adcInterface",
        "//hw/interfaces:gpioInterface",
        "//hw/interfaces:spiInterface",
        "//hw/interfaces:uartInterface",
        "//utils:consts",
        "//utils/time:timeInterface",
    ],
)
