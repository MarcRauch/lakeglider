load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "hw_test_sensors",
    srcs = ["hw_test_sensors.cc"],
    defines = [
        "PICO_STDIO_USB=1",
        "PICO_STDIO_UART=0",
    ],
    deps = [
        "//com/msg:msgs",
        "//hw/interfaces/pico:adcPico",
        "//hw/interfaces/pico:i2cPico",
        "//hw/sensors",
        "//utils/time:timePico",
        "@pico-sdk//src/rp2_common/pico_stdlib:pico_stdlib",
    ],
)
