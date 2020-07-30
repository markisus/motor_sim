#pragma once

#include <SDL.h>

namespace biro {
namespace wrappers {

// Call this after initting SDL and gladLoadGL()
void sdl_imgui_init(SDL_Window* window, SDL_GLContext gl_context);
void sdl_imgui_cleanup();

// RAII wrapper for sdl_imgui_init and sdl_imgui_cleanup
class SdlImguiContext {
   public:
    // non-copyable
    SdlImguiContext(const SdlImguiContext&) = delete;
    SdlImguiContext& operator=(const SdlImguiContext&) = delete;

    // Calls sdl_imgui_init
    SdlImguiContext(SDL_Window* window, SDL_GLContext gl_context);

    // Calls sdl_imgui_cleanup
    ~SdlImguiContext();
};

}  // namespace wrappers
}  // namespace biro
