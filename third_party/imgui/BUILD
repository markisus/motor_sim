package(default_visibility = ["//visibility:public"])

# Users should depend on "imgui_glfw" or "imgui_sdl"
# depending on which window library you want to use

cc_library(
    name = "imgui_glfw",
    srcs = [
        "imgui_impl_glfw.cpp",
        "imgui_impl_opengl3.cpp",
        "imgui_impl_glfw.h",
        "imgui_impl_opengl3.h",
    ],
    hdrs = [
        "imgui_impl_glfw.h",
        "imgui_impl_opengl3.h",
    ],
    deps = [
        ":imgui_base",
        "//third_party/glad:glad",
        "//third_party/glfw:glfw",
    ],
    strip_include_prefix = "/third_party/imgui",
)

cc_library(
    name = "imgui_sdl",
    srcs = [
        "imgui_impl_sdl.cpp",
        "imgui_impl_sdl.h",
        "imgui_impl_opengl3.cpp",
        "imgui_impl_opengl3.h",
    ],
    hdrs = [
        "imgui_impl_opengl3.h",
        "imgui_impl_sdl.h",
    ],
    deps = [
        ":imgui_base",
        "//third_party/glad:glad",
        "//third_party/sdl:sdl",
    ],
    strip_include_prefix = "/third_party/imgui",
)

cc_library(
    name = "imgui_base",
    srcs = [
        "imgui.cpp",
        "imgui_demo.cpp",
        "imgui_draw.cpp",
        "imgui_widgets.cpp",
        "imgui.h",
        "imgui_internal.h",
        "imconfig.h",
        "imstb_rectpack.h",
        "imstb_textedit.h",
        "imstb_truetype.h"
    ],
    hdrs = [
        "imconfig.h",
        "imgui.h",
        "imgui_internal.h",
        "imstb_textedit.h",
    ],
    strip_include_prefix = "/third_party/imgui",
)
