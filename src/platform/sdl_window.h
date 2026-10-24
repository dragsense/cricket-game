#pragma once
// SDL3 window + input. The only place (besides imgui_platform) that touches SDL.

#include "core/events.h"
#include "core/input.h"

#include <string>

struct SDL_Window;

namespace platform {

struct WindowDesc {
    std::string title = "Cricket";
    int width = 1600;
    int height = 900;
    bool resizable = true;
};

class Window {
public:
    bool create(const WindowDesc& desc);
    void destroy();

    // Drains SDL events into `input` and `events`. Call input.beginFrame() first.
    void pumpEvents(core::InputState& input, core::EventQueue& events);

    // Relative mouse mode hides the cursor and reports unbounded deltas
    // (used while mouse-looking with the free-fly camera).
    void setRelativeMouse(bool enabled);

    void* nativeHandle() const;   // HWND on Windows; what Filament needs for a swap chain
    int pixelWidth() const;
    int pixelHeight() const;
    float refreshRate() const;     // Hz of the display the window is on (60 if unknown)
    SDL_Window* sdlWindow() const { return window_; }

private:
    SDL_Window* window_ = nullptr;
    bool relativeMouse_ = false;
};

// Seconds-resolution sleep used by the frame limiter (platform specific timing).
void preciseSleep(double seconds);

} // namespace platform
