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

cc_binary(
    name = "hw_test_actuators",
    srcs = ["hw_test_actuators.cc"],
    defines = [
        "PICO_STDIO_USB=1",
        "PICO_STDIO_UART=0",
    ],
    deps = [
        "//hw/actuators",
        "//hw/interfaces/pico:adcPico",
        "//hw/interfaces/pico:gpioPico",
        "//hw/interfaces/pico:spiPico",
        "//hw/interfaces/pico:uartPico",
        "//hw/sensors",
        "//utils/time:timePico",
        "@pico-sdk//src/rp2_common/pico_stdlib:pico_stdlib",
    ],
)

cc_binary(
    name = "hw_test_com",
    srcs = ["hw_test_com.cc"],
    defines = [
        "PICO_STDIO_USB=1",
        "PICO_STDIO_UART=0",
    ],
    deps = [
        "//hw/interfaces/drivers:canInterface",
        "//hw/interfaces/pico:spiPico",
        "//utils/time:timePico",
        "@pico-sdk//src/rp2_common/pico_stdlib:pico_stdlib",
    ],
)
