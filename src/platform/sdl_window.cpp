#include "platform/sdl_window.h"

#include "core/log.h"
#include "platform/imgui_platform.h"

#include <SDL3/SDL.h>

#include <algorithm>

namespace platform {

namespace {

core::Key translateKey(SDL_Scancode sc) {
    using core::Key;
    if (sc >= SDL_SCANCODE_A && sc <= SDL_SCANCODE_Z)
        return static_cast<Key>(static_cast<int>(Key::A) + (sc - SDL_SCANCODE_A));
    if (sc >= SDL_SCANCODE_1 && sc <= SDL_SCANCODE_9)
        return static_cast<Key>(static_cast<int>(Key::Num1) + (sc - SDL_SCANCODE_1));
    if (sc >= SDL_SCANCODE_F1 && sc <= SDL_SCANCODE_F12)
        return static_cast<Key>(static_cast<int>(Key::F1) + (sc - SDL_SCANCODE_F1));
    switch (sc) {
        case SDL_SCANCODE_0: return Key::Num0;
        case SDL_SCANCODE_SPACE: return Key::Space;
        case SDL_SCANCODE_ESCAPE: return Key::Escape;
        case SDL_SCANCODE_TAB: return Key::Tab;
        case SDL_SCANCODE_RETURN: return Key::Enter;
        case SDL_SCANCODE_BACKSPACE: return Key::Backspace;
        case SDL_SCANCODE_LSHIFT: return Key::LeftShift;
        case SDL_SCANCODE_RSHIFT: return Key::RightShift;
        case SDL_SCANCODE_LCTRL: return Key::LeftCtrl;
        case SDL_SCANCODE_RCTRL: return Key::RightCtrl;
        case SDL_SCANCODE_LALT: return Key::LeftAlt;
        case SDL_SCANCODE_RALT: return Key::RightAlt;
        case SDL_SCANCODE_UP: return Key::Up;
        case SDL_SCANCODE_DOWN: return Key::Down;
        case SDL_SCANCODE_LEFT: return Key::Left;
        case SDL_SCANCODE_RIGHT: return Key::Right;
        default: return Key::Unknown;
    }
}

bool translateButton(Uint8 b, core::MouseButton& out) {
    switch (b) {
        case SDL_BUTTON_LEFT: out = core::MouseButton::Left; return true;
        case SDL_BUTTON_RIGHT: out = core::MouseButton::Right; return true;
        case SDL_BUTTON_MIDDLE: out = core::MouseButton::Middle; return true;
        default: return false;
    }
}

} // namespace

bool Window::create(const WindowDesc& desc) {
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS)) {
        LOG_ERROR("SDL_Init failed: {}", SDL_GetError());
        return false;
    }
    // Created hidden so it can be sized and placed before it first appears.
    SDL_WindowFlags flags = SDL_WINDOW_HIGH_PIXEL_DENSITY | SDL_WINDOW_HIDDEN;
    if (desc.resizable) flags |= SDL_WINDOW_RESIZABLE;
    window_ = SDL_CreateWindow(desc.title.c_str(), desc.width, desc.height, flags);
    if (!window_) {
        LOG_ERROR("SDL_CreateWindow failed: {}", SDL_GetError());
        return false;
    }
    placeOnScreen(desc);  // also the size it restores to when un-maximized
    SDL_ShowWindow(window_);
    if (desc.dock == "maximized") SDL_MaximizeWindow(window_);  // normal window: movable, minimizable, resizable
    if (desc.dock == "fullscreen") SDL_SetWindowFullscreen(window_, true);  // borderless, desktop resolution
    SDL_SyncWindow(window_);

    int w = 0, h = 0;
    SDL_GetWindowSize(window_, &w, &h);
    LOG_INFO("window {}x{} ({}x{} px), dock '{}', display {:.0f} Hz", w, h, pixelWidth(), pixelHeight(),
             desc.dock, refreshRate());
    return true;
}

void Window::placeOnScreen(const WindowDesc& desc) {
    SDL_Rect usable{};
    if (!SDL_GetDisplayUsableBounds(SDL_GetPrimaryDisplay(), &usable)) return;  // keep defaults

    // Window size excludes the title bar and borders; leave room for them.
    int top = 0, left = 0, bottom = 0, right = 0;
    SDL_GetWindowBordersSize(window_, &top, &left, &bottom, &right);
    int availW = usable.w - left - right;
    int availH = usable.h - top - bottom;

    int x = 0, y = 0, w = desc.width, h = desc.height;
    if (desc.dock == "left" || desc.dock == "right") {
        // Half the screen, full height: code editor on the other half.
        w = availW / 2 - (left + right);
        h = availH;
        x = usable.x + left + (desc.dock == "right" ? usable.w / 2 : 0);
        y = usable.y + top;
    } else {
        // "center": requested size, shrunk (keeping aspect) to fit the screen.
        float maxW = availW * desc.maxScreenFraction, maxH = availH * desc.maxScreenFraction;
        float scale = std::min({1.0f, maxW / static_cast<float>(w), maxH / static_cast<float>(h)});
        w = static_cast<int>(static_cast<float>(w) * scale);
        h = static_cast<int>(static_cast<float>(h) * scale);
        x = usable.x + (usable.w - w) / 2;
        y = usable.y + (usable.h - h) / 2;
    }
    SDL_SetWindowSize(window_, w, h);
    SDL_SetWindowPosition(window_, x, y);
}

void Window::destroy() {
    if (window_) SDL_DestroyWindow(window_);
    window_ = nullptr;
    SDL_Quit();
}

void Window::pumpEvents(core::InputState& input, core::EventQueue& events) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        imguiProcessEvent(&e);
        switch (e.type) {
            case SDL_EVENT_QUIT:
            case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
                events.push({core::EventType::Quit});
                break;
            case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
                events.push({core::EventType::WindowResized, e.window.data1, e.window.data2});
                break;
            case SDL_EVENT_WINDOW_FOCUS_LOST:
                events.push({core::EventType::FocusLost});
                // Avoid keys getting "stuck" down while unfocused.
                for (int k = 0; k < static_cast<int>(core::Key::Count); ++k)
                    input.setKey(static_cast<core::Key>(k), false);
                break;
            case SDL_EVENT_WINDOW_FOCUS_GAINED:
                events.push({core::EventType::FocusGained});
                break;
            case SDL_EVENT_KEY_DOWN:
            case SDL_EVENT_KEY_UP:
                if (!e.key.repeat) input.setKey(translateKey(e.key.scancode), e.type == SDL_EVENT_KEY_DOWN);
                break;
            case SDL_EVENT_MOUSE_BUTTON_DOWN:
            case SDL_EVENT_MOUSE_BUTTON_UP: {
                core::MouseButton b;
                if (translateButton(e.button.button, b))
                    input.setMouseButton(b, e.type == SDL_EVENT_MOUSE_BUTTON_DOWN);
                break;
            }
            case SDL_EVENT_MOUSE_MOTION:
                input.mouseX = e.motion.x;
                input.mouseY = e.motion.y;
                input.mouseDx += e.motion.xrel;
                input.mouseDy += e.motion.yrel;
                break;
            case SDL_EVENT_MOUSE_WHEEL:
                input.wheel += e.wheel.y;
                break;
            default:
                break;
        }
    }
    input.setKey(core::Key::Unknown, false);  // unmapped keys all land here; keep it clear
}

void Window::setRelativeMouse(bool enabled) {
    if (enabled == relativeMouse_) return;
    relativeMouse_ = enabled;
    SDL_SetWindowRelativeMouseMode(window_, enabled);
}

void* Window::nativeHandle() const {
#if defined(_WIN32)
    return SDL_GetPointerProperty(SDL_GetWindowProperties(window_), SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr);
#else
    return nullptr;  // other platforms arrive with their milestones
#endif
}

int Window::pixelWidth() const {
    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(window_, &w, &h);
    return w;
}

int Window::pixelHeight() const {
    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(window_, &w, &h);
    return h;
}

float Window::refreshRate() const {
    const SDL_DisplayMode* mode = SDL_GetCurrentDisplayMode(SDL_GetDisplayForWindow(window_));
    return (mode && mode->refresh_rate > 1.0f) ? mode->refresh_rate : 60.0f;
}

void preciseSleep(double seconds) {
    if (seconds > 0) SDL_DelayPrecise(static_cast<Uint64>(seconds * 1e9));
}

} // namespace platform
