cmake_minimum_required(VERSION 3.28)
project(CricketGame LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Everything lands next to the exe so it can find config/ and assets/.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(CRICKET_BUILD_TESTS "Build unit tests" ON)

include(cmake/dependencies.cmake)
include(cmake/Filament.cmake)

# Warnings for our own code only (third-party targets keep their defaults).
function(cricket_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8)
        target_compile_definitions(${target} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

# ---- math: header-only GLM wrapper ---------------------------------------------
add_library(cricket_math INTERFACE)
target_include_directories(cricket_math INTERFACE src)
target_link_libraries(cricket_math INTERFACE glm::glm)
target_compile_definitions(cricket_math INTERFACE GLM_FORCE_SILENT_WARNINGS GLM_ENABLE_EXPERIMENTAL)

# ---- core: no platform or render dependencies -----------------------------------
add_library(cricket_core STATIC
    src/core/config.cpp
    src/core/input.cpp
    src/core/app.cpp
    src/core/file_watcher.cpp
)
target_include_directories(cricket_core PUBLIC src)
target_link_libraries(cricket_core PUBLIC cricket_math spdlog::spdlog nlohmann_json::nlohmann_json PRIVATE efsw-static)
cricket_warnings(cricket_core)

# ---- rig: ozz-animation skeletons built from config ---------------------------
add_library(cricket_rig STATIC
    src/rig/skeleton_def.cpp
    src/rig/rig.cpp
)
target_link_libraries(cricket_rig PUBLIC cricket_core PRIVATE ozz_animation ozz_animation_offline)
cricket_warnings(cricket_rig)

# ---- ik: joint limits (own code) + ozz two-bone / aim IK ---------------------------
add_library(cricket_ik STATIC
    src/ik/joint_limits.cpp
    src/ik/limb_ik.cpp
)
target_link_libraries(cricket_ik PUBLIC cricket_rig PRIVATE ozz_animation)
cricket_warnings(cricket_ik)

# ---- motion: procedural body motion (own code) ------------------------------------
add_library(cricket_motion STATIC
    src/motion/body_reposition.cpp
    src/motion/foot_stepper.cpp
    src/motion/reach.cpp
    src/motion/walk_controller.cpp
)
target_link_libraries(cricket_motion PUBLIC cricket_core)
cricket_warnings(cricket_motion)

# ---- physics: Jolt behind our own thin interface; our sphere solver; ball setup ----
add_library(cricket_physics STATIC
    src/physics/core/physics_world.cpp
    src/physics/sim/colliders.cpp
    src/physics/sim/forces.cpp
    src/physics/sim/sphere_solver.cpp
    src/physics/ball/ball_setup.cpp
)
target_link_libraries(cricket_physics PUBLIC cricket_core PRIVATE Jolt)
cricket_warnings(cricket_physics)

# ---- game executable --------------------------------------------------------
add_executable(CricketGame
    src/main.cpp
    src/game/cricket_app.cpp
    src/game/test_scene.cpp
    src/game/world.cpp
    src/platform/sdl_window.cpp
    src/platform/imgui_platform.cpp
    src/render/debug_draw.cpp
    src/render/mesh_gen.cpp
    src/render/filament/filament_renderer.cpp
    src/render/filament/imgui_filament.cpp
    src/camera/debug_camera.cpp
    src/debug/debug_overlay.cpp
    src/match/sim_world.cpp
    src/agents/fielder.cpp
    src/game/fielder_visual.cpp
    src/debug/skeleton_debug.cpp
    src/debug/axis_gizmo.cpp
)
target_link_libraries(CricketGame PRIVATE cricket_core cricket_rig cricket_ik cricket_motion cricket_physics SDL3::SDL3 imgui EnTT::EnTT filament_sdk)
cricket_warnings(CricketGame)
# Debug builds read config/ straight from the source tree and hot-reload it.
target_compile_definitions(CricketGame PRIVATE $<$<CONFIG:Debug>:CRICKET_DEV_CONFIG_DIR="${CMAKE_SOURCE_DIR}/config">)
if(WIN32)
    set_target_properties(CricketGame PROPERTIES WIN32_EXECUTABLE $<CONFIG:Release>)
endif()

# Compile .mat materials with Filament's matc and copy runtime data next to the exe.
cricket_compile_materials(CricketGame
    ${CMAKE_SOURCE_DIR}/assets/materials/lit.mat
    ${CMAKE_SOURCE_DIR}/assets/materials/debug_line.mat
    ${CMAKE_SOURCE_DIR}/assets/materials/ui.mat
    ${CMAKE_SOURCE_DIR}/assets/materials/ball.mat
)
# Config is synced on every build (not only when the exe relinks), so editing
# a .json and running the build is enough to see the change.
add_custom_target(CricketGame_config ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different ${CMAKE_SOURCE_DIR}/config ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/config
    COMMENT "Sync config/"
    VERBATIM)
add_dependencies(CricketGame CricketGame_config)
add_custom_command(TARGET CricketGame POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:SDL3::SDL3> $<TARGET_FILE_DIR:CricketGame>
    VERBATIM)

# ---- tests --------------------------------------------------------------------
if(CRICKET_BUILD_TESTS)
    enable_testing()
    add_executable(math_tests tests/test_main.cpp tests/test_math.cpp)
    target_link_libraries(math_tests PRIVATE cricket_math doctest::doctest)
    cricket_warnings(math_tests)
    add_test(NAME math_tests COMMAND math_tests)

    add_executable(rig_tests tests/test_main.cpp tests/test_rig.cpp)
    target_link_libraries(rig_tests PRIVATE cricket_rig doctest::doctest)
    target_compile_definitions(rig_tests PRIVATE CRICKET_CONFIG_DIR="${CMAKE_SOURCE_DIR}/config")
    cricket_warnings(rig_tests)
    add_test(NAME rig_tests COMMAND rig_tests)

    add_executable(ik_tests tests/test_main.cpp tests/test_ik.cpp)
    target_link_libraries(ik_tests PRIVATE cricket_ik doctest::doctest)
    target_compile_definitions(ik_tests PRIVATE CRICKET_CONFIG_DIR="${CMAKE_SOURCE_DIR}/config")
    cricket_warnings(ik_tests)
    add_test(NAME ik_tests COMMAND ik_tests)

    add_executable(motion_tests tests/test_main.cpp tests/test_reposition.cpp tests/test_stepping.cpp tests/test_walk.cpp tests/test_reach.cpp)
    target_link_libraries(motion_tests PRIVATE cricket_motion doctest::doctest)
    cricket_warnings(motion_tests)
    add_test(NAME motion_tests COMMAND motion_tests)

    add_executable(agents_tests tests/test_main.cpp tests/test_fielder.cpp tests/test_ball_feeder.cpp src/agents/fielder.cpp
        src/game/ball_feeder.cpp)
    target_link_libraries(agents_tests PRIVATE cricket_ik cricket_motion doctest::doctest)
    target_include_directories(agents_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_compile_definitions(agents_tests PRIVATE CRICKET_CONFIG_DIR="${CMAKE_SOURCE_DIR}/config")
    cricket_warnings(agents_tests)
    add_test(NAME agents_tests COMMAND agents_tests)

    add_executable(physics_tests tests/test_main.cpp tests/test_physics_world.cpp tests/test_sphere_solver.cpp)
    target_link_libraries(physics_tests PRIVATE cricket_physics doctest::doctest)
    target_compile_definitions(physics_tests PRIVATE CRICKET_CONFIG_DIR="${CMAKE_SOURCE_DIR}/config")
    cricket_warnings(physics_tests)
    add_test(NAME physics_tests COMMAND physics_tests)
endif()
