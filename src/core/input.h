#pragma once
// Platform-independent input state and action mapping. src/platform fills an
// InputState from SDL events; game code asks the ActionMap about actions
// ("StepForward") rather than physical keys.

#include <cstdint>
#include <string_view>

namespace core {

class Config;

enum class Key : uint8_t {
    Unknown,
    A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
    Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
    Space, Escape, Tab, Enter, Backspace,
    LeftShift, RightShift, LeftCtrl, RightCtrl, LeftAlt, RightAlt,
    Up, Down, Left, Right,
    F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12,
    Count
};

enum class MouseButton : uint8_t { Left, Right, Middle, Count };

Key keyFromName(std::string_view name);  // "W", "LeftShift", "F1"...; Unknown if not found

// Raw device state for one rendered frame. "Pressed" flags and deltas are
// edge/accumulated values and are cleared by beginFrame().
struct InputState {
    bool keyDown[static_cast<int>(Key::Count)] = {};
    bool keyPressed[static_cast<int>(Key::Count)] = {};
    bool mouseDown[static_cast<int>(MouseButton::Count)] = {};
    bool mousePressed[static_cast<int>(MouseButton::Count)] = {};
    float mouseX = 0, mouseY = 0;    // window pixels
    float mouseDx = 0, mouseDy = 0;  // accumulated motion this frame
    float wheel = 0;                 // accumulated vertical wheel this frame

    // Set by the UI layer when ImGui wants the keyboard/mouse, so game
    // controls ignore input aimed at debug windows.
    bool uiWantsKeyboard = false;
    bool uiWantsMouse = false;

    void beginFrame();
    void setKey(Key k, bool down);
    void setMouseButton(MouseButton b, bool down);

    bool down(Key k) const { return keyDown[static_cast<int>(k)]; }
    bool pressed(Key k) const { return keyPressed[static_cast<int>(k)]; }
    bool down(MouseButton b) const { return mouseDown[static_cast<int>(b)]; }
    bool pressed(MouseButton b) const { return mousePressed[static_cast<int>(b)]; }
};

enum class Action : uint8_t {
    ToggleOverlay, ToggleDebugDraw, ReloadConfig, Quit,
    StepForward, StepBack, StepLeft, StepRight,
    Run, Jog,  // hold to run / jog; otherwise walk
    ToggleMouseLook,
    ClearReach,  // drop the hand target (left-drag moves the ball, which is the target)
    Pick,              // reach for the ball, grip it, carry it back
    ThrowBall,         // release the ball from the batting position toward the fielder
    LaunchBall1, LaunchBall2, LaunchBall3, LaunchBall4, LaunchBall5,  // ball physics presets
    Count
};

// Two bindings per action, loaded from config/input.json:
//   { "bindings": { "StepForward": ["W"], ... } }
class ActionMap {
public:
    ActionMap();  // sensible defaults
    void load(const Config& config);

    bool down(const InputState& in, Action a) const;
    bool pressed(const InputState& in, Action a) const;

private:
    static constexpr int kSlots = 2;
    Key bindings_[static_cast<int>(Action::Count)][kSlots] = {};
};

const char* actionName(Action a);

} // namespace core
