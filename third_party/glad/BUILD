package(default_visibility = ["//visibility:public"])

cc_library(
    name = "khr",
    hdrs = ["khrplatform.h"],
    include_prefix = "KHR",
    strip_include_prefix = "/third_party/glad")

cc_library(
    name = "glad",
    srcs = ["glad.c"],
    hdrs = ["glad.h"],
    deps = [":khr"],
    strip_include_prefix = "/third_party/glad",
    include_prefix = "glad",
)
