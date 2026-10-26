#include "core/input.h"

#include "core/config.h"
#include "core/log.h"

#include <cstring>

namespace core {

namespace {

// Names for keys after the letters and digits (which are generated).
struct NamedKey {
    const char* name;
    Key key;
};
constexpr NamedKey kNamedKeys[] = {
    {"Space", Key::Space}, {"Escape", Key::Escape}, {"Tab", Key::Tab}, {"Enter", Key::Enter},
    {"Backspace", Key::Backspace}, {"LeftShift", Key::LeftShift}, {"RightShift", Key::RightShift},
    {"LeftCtrl", Key::LeftCtrl}, {"RightCtrl", Key::RightCtrl}, {"LeftAlt", Key::LeftAlt},
    {"RightAlt", Key::RightAlt}, {"Up", Key::Up}, {"Down", Key::Down}, {"Left", Key::Left},
    {"Right", Key::Right}, {"F1", Key::F1}, {"F2", Key::F2}, {"F3", Key::F3}, {"F4", Key::F4},
    {"F5", Key::F5}, {"F6", Key::F6}, {"F7", Key::F7}, {"F8", Key::F8}, {"F9", Key::F9},
    {"F10", Key::F10}, {"F11", Key::F11}, {"F12", Key::F12},
};

constexpr const char* kActionNames[] = {
    "ToggleOverlay", "ToggleDebugDraw", "ReloadConfig", "Quit",
    "StepForward", "StepBack", "StepLeft", "StepRight",
    "Run", "Jog",
    "ToggleMouseLook",
    "ClearReach",
    "Pick",
    "ThrowBall",
    "LaunchBall1", "LaunchBall2", "LaunchBall3", "LaunchBall4", "LaunchBall5",
};
static_assert(sizeof(kActionNames) / sizeof(kActionNames[0]) == static_cast<int>(Action::Count));

} // namespace

Key keyFromName(std::string_view name) {
    if (name.size() == 1) {
        char c = name[0];
        if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
        if (c >= 'A' && c <= 'Z') return static_cast<Key>(static_cast<int>(Key::A) + (c - 'A'));
        if (c >= '0' && c <= '9') return static_cast<Key>(static_cast<int>(Key::Num0) + (c - '0'));
    }
    for (const auto& nk : kNamedKeys)
        if (name == nk.name) return nk.key;
    return Key::Unknown;
}

const char* actionName(Action a) { return kActionNames[static_cast<int>(a)]; }

void InputState::beginFrame() {
    std::memset(keyPressed, 0, sizeof(keyPressed));
    std::memset(mousePressed, 0, sizeof(mousePressed));
    mouseDx = mouseDy = 0;
    wheel = 0;
}

void InputState::setKey(Key k, bool isDown) {
    int i = static_cast<int>(k);
    if (isDown && !keyDown[i]) keyPressed[i] = true;
    keyDown[i] = isDown;
}

void InputState::setMouseButton(MouseButton b, bool isDown) {
    int i = static_cast<int>(b);
    if (isDown && !mouseDown[i]) mousePressed[i] = true;
    mouseDown[i] = isDown;
}

ActionMap::ActionMap() {
    auto bind = [this](Action a, Key k0, Key k1 = Key::Unknown) {
        bindings_[static_cast<int>(a)][0] = k0;
        bindings_[static_cast<int>(a)][1] = k1;
    };
    bind(Action::ToggleOverlay, Key::F1);
    bind(Action::ToggleDebugDraw, Key::F2);
    bind(Action::ReloadConfig, Key::F5);
    bind(Action::Quit, Key::Escape);
    bind(Action::StepForward, Key::W);
    bind(Action::StepBack, Key::S);
    bind(Action::StepLeft, Key::A);
    bind(Action::StepRight, Key::D);
    bind(Action::Run, Key::LeftShift, Key::RightShift);
    bind(Action::Jog, Key::LeftCtrl, Key::RightCtrl);
    bind(Action::ToggleMouseLook, Key::Tab);
    bind(Action::ClearReach, Key::R);
    bind(Action::Pick, Key::Space);
    bind(Action::ThrowBall, Key::T);
    bind(Action::LaunchBall1, Key::Num1);
    bind(Action::LaunchBall2, Key::Num2);
    bind(Action::LaunchBall3, Key::Num3);
    bind(Action::LaunchBall4, Key::Num4);
    bind(Action::LaunchBall5, Key::Num5);
}

void ActionMap::load(const Config& config) {
    auto b = config.root().find("bindings");
    if (b == config.root().end() || !b->is_object()) return;
    for (int a = 0; a < static_cast<int>(Action::Count); ++a) {
        auto keys = b->find(kActionNames[a]);
        if (keys == b->end() || !keys->is_array()) continue;
        for (int s = 0; s < kSlots; ++s) {
            Key k = Key::Unknown;
            if (s < static_cast<int>(keys->size()) && (*keys)[s].is_string()) {
                const auto& name = (*keys)[s].get_ref<const std::string&>();
                k = keyFromName(name);
                if (k == Key::Unknown) LOG_WARN("input: unknown key '{}' for {}", name, kActionNames[a]);
            }
            bindings_[a][s] = k;
        }
    }
}

bool ActionMap::down(const InputState& in, Action a) const {
    if (in.uiWantsKeyboard) return false;
    for (Key k : bindings_[static_cast<int>(a)])
        if (k != Key::Unknown && in.down(k)) return true;
    return false;
}

bool ActionMap::pressed(const InputState& in, Action a) const {
    if (in.uiWantsKeyboard) return false;
    for (Key k : bindings_[static_cast<int>(a)])
        if (k != Key::Unknown && in.pressed(k)) return true;
    return false;
}

} // namespace core
