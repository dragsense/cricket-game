# Every third-party library and its pinned version, in one place.
# scripts/setup_windows.ps1 reads FILAMENT_VERSION from this file, so keep it
# in the one-line set(NAME "value") form.

set(FILAMENT_VERSION "v1.77.2")          # prebuilt SDK, downloaded by the setup script
set(SDL3_VERSION     "release-3.4.16")
set(GLM_VERSION      "1.0.1")
set(IMGUI_VERSION    "v1.92.9b-docking")
set(JSON_VERSION     "v3.12.0")
set(SPDLOG_VERSION   "v1.15.3")
set(ENTT_VERSION     "v3.15.0")
set(DOCTEST_VERSION  "v2.5.2")
# Planned, not integrated yet:
#   Milestone 1: ozz-animation (skeletons, two-bone/aim IK, blending, skinning),
#                ImGuizmo (draggable IK targets)
#   Later:       Jolt Physics (player body collisions)
# Model loading uses Filament's gltfio (ships with the Filament SDK).

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

# --- SDL3 (shared library, copied next to the exe) ---
set(SDL_SHARED ON CACHE BOOL "" FORCE)
set(SDL_STATIC OFF CACHE BOOL "" FORCE)
set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG ${SDL3_VERSION}
    GIT_SHALLOW TRUE
    EXCLUDE_FROM_ALL)

# --- GLM ---
set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLM_BUILD_LIBRARY OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG ${GLM_VERSION}
    GIT_SHALLOW TRUE
    EXCLUDE_FROM_ALL)

# --- Dear ImGui (docking branch; has no CMake of its own) ---
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG ${IMGUI_VERSION}
    GIT_SHALLOW TRUE)

# --- nlohmann/json (config files) ---
FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/${JSON_VERSION}/json.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    EXCLUDE_FROM_ALL)

# --- spdlog (logging) ---
set(SPDLOG_USE_STD_FORMAT ON CACHE BOOL "" FORCE)  # C++20 <format>, no bundled fmt
FetchContent_Declare(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG ${SPDLOG_VERSION}
    GIT_SHALLOW TRUE
    EXCLUDE_FROM_ALL)

# --- doctest ---
set(DOCTEST_WITH_TESTS OFF CACHE BOOL "" FORCE)
set(DOCTEST_NO_INSTALL ON CACHE BOOL "" FORCE)
FetchContent_Declare(doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG ${DOCTEST_VERSION}
    GIT_SHALLOW TRUE
    EXCLUDE_FROM_ALL)

# --- EnTT (entity-component system for game objects) ---
FetchContent_Declare(EnTT
    GIT_REPOSITORY https://github.com/skypjack/entt.git
    GIT_TAG ${ENTT_VERSION}
    GIT_SHALLOW TRUE
    EXCLUDE_FROM_ALL)

FetchContent_MakeAvailable(SDL3 glm imgui nlohmann_json spdlog EnTT doctest)

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC SDL3::SDL3)
