package(default_visibility = ["//visibility:public"])

cc_library(
    name = "global_debug",
    hdrs = ["global_debug.h"],
    srcs = [
        "global_debug.cpp",
        "global_debug.h",
    ],
    strip_include_prefix = "/global_debug"
)

cc_binary(
    name = "example",
    srcs = ["example.cpp"],
    deps = [":global_debug"])
