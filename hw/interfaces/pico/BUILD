load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "adcPico",
    srcs = ["AdcPico.cc"],
    hdrs = [
        "AdcPico.hh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//hw:pins",
        "//hw/interfaces:adcInterface",
        "@pico-sdk//src/rp2_common/hardware_adc",
    ],
)

cc_library(
    name = "i2cPico",
    srcs = ["I2cPico.cc"],
    hdrs = [
        "I2cPico.hh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//hw:pins",
        "//hw/interfaces:i2cInterface",
        "@pico-sdk//src/rp2_common/hardware_i2c",
        "@pico-sdk//src/rp2_common/pico_stdlib:pico_stdlib",
    ],
)

cc_library(
    name = "spiPico",
    srcs = ["SpiPico.cc"],
    hdrs = [
        "SpiPico.hh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//hw:pins",
        "//hw/interfaces:spiInterface",
        "@pico-sdk//src/rp2_common/hardware_spi",
        "@pico-sdk//src/rp2_common/pico_stdlib:pico_stdlib",
    ],
)

cc_library(
    name = "uartPico",
    srcs = ["UartPico.cc"],
    hdrs = [
        "UartPico.hh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//hw:pins",
        "//hw/interfaces:uartInterface",
        "@pico-sdk//src/rp2_common/hardware_uart",
        "@pico-sdk//src/rp2_common/pico_stdlib:pico_stdlib",
    ],
)

cc_library(
    name = "gpioPico",
    srcs = ["GpioPico.cc"],
    hdrs = [
        "GpioPico.hh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//hw:pins",
        "//hw/interfaces:gpioInterface",
        "@pico-sdk//src/rp2_common/pico_stdlib:pico_stdlib",
    ],
)
