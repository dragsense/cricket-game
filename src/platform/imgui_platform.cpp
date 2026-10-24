#include "platform/imgui_platform.h"

#include "platform/sdl_window.h"

#include <SDL3/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>

namespace platform {

namespace {
bool gInitialised = false;
}

bool imguiPlatformInit(Window& window) {
    // "Other" renderer: our Filament-based renderer draws the ImGui output.
    gInitialised = ImGui_ImplSDL3_InitForOther(window.sdlWindow());
    return gInitialised;
}

void imguiPlatformShutdown() {
    if (gInitialised) ImGui_ImplSDL3_Shutdown();
    gInitialised = false;
}

void imguiPlatformNewFrame() { ImGui_ImplSDL3_NewFrame(); }

void imguiProcessEvent(const void* sdlEvent) {
    if (gInitialised) ImGui_ImplSDL3_ProcessEvent(static_cast<const SDL_Event*>(sdlEvent));
}

} // namespace platform
