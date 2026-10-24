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
set(OZZ_VERSION      "0.16.0")
# Planned, not integrated yet:
#   Milestone 1: ImGuizmo (draggable IK targets, Step 1.2)
#   Later:       Jolt Physics (player body collisions)
# Model loading uses Filament's gltfio (ships with the Filament SDK).

# Sources are cloned ONCE into third_party/deps/<name>-<version>/ and shared by
# every build folder (debug, release). CMake never downloads them itself, so
# reconfiguring or deleting build/ never re-clones or forces a full rebuild.
# Changing a version above clones the new version into a new folder.
include(FetchContent)
set(CRICKET_DEPS_DIR ${CMAKE_SOURCE_DIR}/third_party/deps)

# cricket_dependency(<FetchContent name> <git repo> <tag>)
function(cricket_dependency name repo tag)
    set(dir ${CRICKET_DEPS_DIR}/${name}-${tag})
    if(NOT EXISTS ${dir}/.cricket-complete)
        find_package(Git REQUIRED)
        file(REMOVE_RECURSE ${dir})
        message(STATUS "Downloading ${name} ${tag} (one-time) ...")
        execute_process(
            COMMAND ${GIT_EXECUTABLE} -c advice.detachedHead=false clone --quiet --depth 1 --branch ${tag} ${repo} ${dir}
            RESULT_VARIABLE result)
        if(NOT result EQUAL 0)
            message(FATAL_ERROR "Failed to clone ${repo} at ${tag}")
        endif()
        file(TOUCH ${dir}/.cricket-complete)
    endif()
    # Tell FetchContent the source is already present: no download step at all.
    string(TOUPPER ${name} upper)
    set(FETCHCONTENT_SOURCE_DIR_${upper} ${dir} PARENT_SCOPE)
    FetchContent_Declare(${name} GIT_REPOSITORY ${repo} GIT_TAG ${tag} EXCLUDE_FROM_ALL)
endfunction()

# --- SDL3 (shared library, copied next to the exe) ---
set(SDL_SHARED ON CACHE BOOL "" FORCE)
set(SDL_STATIC OFF CACHE BOOL "" FORCE)
set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
cricket_dependency(SDL3 https://github.com/libsdl-org/SDL.git ${SDL3_VERSION})

# --- GLM ---
set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLM_BUILD_LIBRARY OFF CACHE BOOL "" FORCE)
cricket_dependency(glm https://github.com/g-truc/glm.git ${GLM_VERSION})

# --- Dear ImGui (docking branch; has no CMake of its own) ---
cricket_dependency(imgui https://github.com/ocornut/imgui.git ${IMGUI_VERSION})

# --- nlohmann/json (config files) ---
cricket_dependency(nlohmann_json https://github.com/nlohmann/json.git ${JSON_VERSION})

# --- spdlog (logging) ---
set(SPDLOG_USE_STD_FORMAT ON CACHE BOOL "" FORCE)  # C++20 <format>, no bundled fmt
cricket_dependency(spdlog https://github.com/gabime/spdlog.git ${SPDLOG_VERSION})

# --- doctest ---
set(DOCTEST_WITH_TESTS OFF CACHE BOOL "" FORCE)
set(DOCTEST_NO_INSTALL ON CACHE BOOL "" FORCE)
cricket_dependency(doctest https://github.com/doctest/doctest.git ${DOCTEST_VERSION})

# --- EnTT (entity-component system for game objects) ---
cricket_dependency(EnTT https://github.com/skypjack/entt.git ${ENTT_VERSION})

# --- ozz-animation (runtime skeletons, local-to-model, two-bone/aim IK) ---
# Runtime + offline (RawSkeleton/SkeletonBuilder) libraries only. DLL CRT to
# match the rest of the build.
set(ozz_build_tools OFF CACHE BOOL "" FORCE)
set(ozz_build_fbx OFF CACHE BOOL "" FORCE)
set(ozz_build_gltf OFF CACHE BOOL "" FORCE)
set(ozz_build_samples OFF CACHE BOOL "" FORCE)
set(ozz_build_howtos OFF CACHE BOOL "" FORCE)
set(ozz_build_tests OFF CACHE BOOL "" FORCE)
set(ozz_build_postfix OFF CACHE BOOL "" FORCE)
set(ozz_build_msvc_rt_dll ON CACHE BOOL "" FORCE)
cricket_dependency(ozz_animation https://github.com/guillaumeblanc/ozz-animation.git ${OZZ_VERSION})

FetchContent_MakeAvailable(SDL3 glm imgui nlohmann_json spdlog EnTT doctest ozz_animation)

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC SDL3::SDL3)
