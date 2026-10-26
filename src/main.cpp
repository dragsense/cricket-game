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
    configs.dir = "config";
#ifdef CRICKET_DEV_CONFIG_DIR
    // Dev builds read config straight from the source tree, so edits apply live
    // (the game watches this folder) without any copy step.
    if (std::filesystem::exists(CRICKET_DEV_CONFIG_DIR)) {
        configs.dir = CRICKET_DEV_CONFIG_DIR;
        configs.liveReload = true;
    }
#endif
    configs.engine.load(configs.dir + "/engine.json");
    configs.input.load(configs.dir + "/input.json");
    configs.scene.load(configs.dir + "/scene.json");
    configs.fielderSkeleton.load(configs.dir + "/fielder_skeleton.json");
    configs.fielderJoints.load(configs.dir + "/fielder_joints.json");
    configs.locomotion.load(configs.dir + "/locomotion.json");
    configs.physicsWorld.load(configs.dir + "/physics_world.json");

    game::CricketApp app(std::move(configs));
    return app.run();
}
