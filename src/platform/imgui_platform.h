#pragma once
// Dear ImGui <-> SDL3 platform glue (the official imgui_impl_sdl3 backend).
// Kept in src/platform so SDL stays out of the debug/render layers.

namespace platform {

class Window;

bool imguiPlatformInit(Window& window);   // ImGui context must already exist
void imguiPlatformShutdown();
void imguiPlatformNewFrame();
void imguiProcessEvent(const void* sdlEvent);  // no-op until initialised

} // namespace platform
