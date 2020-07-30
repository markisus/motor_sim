#include "sdl_imgui_context.h"

#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl.h>

namespace biro {
namespace wrappers {

void sdl_imgui_init(SDL_Window* window, SDL_GLContext gl_context) {
    // Setup ImGUi
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(/*glsl_version=*/nullptr);
}

void sdl_imgui_cleanup() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
}

SdlImguiContext::SdlImguiContext(SDL_Window* window, SDL_GLContext gl_context) {
    sdl_imgui_init(window, gl_context);
}

SdlImguiContext::~SdlImguiContext() { sdl_imgui_cleanup(); }

}  // namespace wrappers
}  // namespace biro
