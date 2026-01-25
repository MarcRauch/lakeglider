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
