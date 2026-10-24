// Entry point: load config, run the app.
// Paths are relative to the executable's directory (the build copies config/
// and compiled assets next to it).

#include "core/log.h"
#include "game/cricket_app.h"

#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_main.h>

#include <filesystem>

int main(int /*argc*/, char** /*argv*/) {
    // Work from the exe directory regardless of how the game was launched.
    if (const char* base = SDL_GetBasePath()) std::filesystem::current_path(base);

    game::AppConfigs configs;
    configs.engine.load("config/engine.json");
    configs.input.load("config/input.json");
    configs.scene.load("config/scene.json");

    game::CricketApp app(std::move(configs));
    return app.run();
}
