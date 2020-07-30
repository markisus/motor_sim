#pragma once

#include <SDL.h>

namespace biro {
namespace wrappers {

// Sets up commonly used window options and calls SDL_Init and
// SDL_GL_CreateContext, returning window and gl_context
int sdl_init(const char* title,
             const int width,
             const int height,
             SDL_Window** window,
             SDL_GLContext* gl_context);

// Calls SDL_GL_DeleteContext *window is non-null and SDL_DestroyWindow if
// *gl_context is non-null. Sets *window and *gl_context to be nullptr.
void sdl_cleanup(SDL_Window** window, SDL_GLContext* gl_context);

// RAII wrapper for sdl_init and sdl_cleanup
class SdlContext {
   public:
    // non-copyable
    SdlContext(const SdlContext&) = delete;
    SdlContext& operator=(const SdlContext&) = delete;

    // Calls sdl_init, saving results into window_ and gl_context_
    SdlContext(const char* title, const int width, const int height);

    // Calls sdl_cleanup with &window_ and &gl_context_
    ~SdlContext();

    int status_ = 0;
    SDL_Window* window_ = nullptr;
    SDL_GLContext gl_context_ = nullptr;
};

}  // namespace wrappers
}  // namespace biro
