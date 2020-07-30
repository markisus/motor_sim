package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sdl_context",
    hdrs = ["sdl_context.h"],
    srcs = ["sdl_context.cpp"],
    deps = [
        "//third_party/sdl:sdl",
    ]
)
cc_library(
    name = "sdl_imgui",
    hdrs = ["sdl_imgui.h",
            "sdl_imgui_context.h"],
    srcs = ["sdl_imgui_context.cpp"],
    deps = [
        "//third_party/sdl:sdl",
        "//third_party/imgui:imgui_sdl",
    ]
)
