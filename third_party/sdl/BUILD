package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sdl",
    hdrs = glob(["include/**/*"]),
    deps = select({
        "@bazel_tools//src/conditions:windows": [":sdl_windows"],
        "//conditions:default": [], # linux not supported yet
    }),
    strip_include_prefix = "/third_party/sdl/include",
)

cc_library(
    name = "sdl_windows",
    includes = [
        "include",
    ],
    srcs = glob(["lib/win64/*.lib"]),
    deps = [":sdl_dlls"] # triggers runtime copy of dll into executable dir
)

cc_library(
    name = "sdl_dlls",
    srcs = glob(["lib/win64/*.dll"]),
)


