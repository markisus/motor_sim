package(default_visibility = ["//visibility:public"])

COPTS = select({
    "@bazel_tools//src/conditions:windows": ["/std:c++17"],
    "//conditions:default": ["-std=c++17"],})

# Version is 3.3.7:
# Downloaded from: http://bitbucket.org/eigen/eigen/get/3.3.7.tar.bz2

cc_library(
    name = "eigen",
    srcs = [],
    includes = ['.'],
    hdrs = glob(["Eigen/**", "unsupported/Eigen/**"]),
    strip_include_prefix = "/third_party/eigen",
    copts = COPTS
)
