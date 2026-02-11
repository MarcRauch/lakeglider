load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_binary(
    name = "server_main",
    srcs = ["server_main.cc"],
    deps = [
        ":bus",
        "//utils/logging:logger",
        "@gflags",
    ],
)

cc_library(
    name = "bus",
    srcs = [
        "Client.cc",
        "Server.cc",
    ],
    hdrs = [
        "Client.hh",
        "Server.hh",
    ],
    deps = [
        "//com/msg:msgs",
        "//utils/logging:logger",
        "//utils/time:timeLinux",
        "@asio",
    ],
)

cc_binary(
    name = "can_bridge_main",
    srcs = ["can_bridge_main.cc"],
    deps = [
        ":bus",
        "//hw:pins",
        "//hw/interfaces/drivers:canInterface",
        "//hw/interfaces/rpi:spiRpi",
        "//utils/time:timeLinux",
        "@gflags",
    ],
)
