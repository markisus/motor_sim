#include "sdl_context.h"

#include <stdio.h>

namespace biro {
namespace wrappers {

int sdl_init(const char* title,
             const int width,
             const int height,
             SDL_Window** window,
             SDL_GLContext* gl_context) {
    const int result = SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    if (result != 0) {
        printf("Error: %s\n", SDL_GetError());
        return result;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
                        SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    const SDL_WindowFlags window_flags = (SDL_WindowFlags)(
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    *window = SDL_CreateWindow(title,
                               SDL_WINDOWPOS_CENTERED,
                               SDL_WINDOWPOS_CENTERED,
                               width,
                               height,
                               window_flags);
    *gl_context = SDL_GL_CreateContext(*window);
    SDL_GL_MakeCurrent(*window, *gl_context);
    SDL_GL_SetSwapInterval(1);  // Enable vsync

    return 0;
}

void sdl_cleanup(SDL_Window** window, SDL_GLContext* gl_context) {
    if (*gl_context) {
        SDL_GL_DeleteContext(*gl_context);
        *gl_context = nullptr;
    }
    if (*window) {
        SDL_DestroyWindow(*window);
        *window = nullptr;
    }
    SDL_Quit();
}

SdlContext::SdlContext(const char* title, const int width, const int height) {
    status_ = sdl_init(title, width, height, &window_, &gl_context_);
};

SdlContext::~SdlContext() { sdl_cleanup(&window_, &gl_context_); }

}  // namespace wrappers
}  // namespace biro
