#pragma once

#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl.h>

namespace biro {
namespace wrappers {

inline bool process_sdl_imgui_events(SDL_Window* window) {
    SDL_Event sdl_event;
    while (SDL_PollEvent(&sdl_event)) {
        ImGui_ImplSDL2_ProcessEvent(&sdl_event);
        if (sdl_event.type == SDL_QUIT) {
            return true;
        }
        if (sdl_event.type == SDL_WINDOWEVENT &&
            sdl_event.window.event == SDL_WINDOWEVENT_CLOSE &&
            sdl_event.window.windowID == SDL_GetWindowID(window)) {
            return true;
        }
    }
    return false;
}

inline void sdl_imgui_newframe(SDL_Window* window) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame(window);
    ImGui::NewFrame();
}

} // namespace wrappers
} // namespace biro
