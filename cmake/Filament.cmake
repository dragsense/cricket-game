# Imports the prebuilt Filament SDK from third_party/filament
# (downloaded by scripts/setup_windows.ps1).

set(FILAMENT_DIR ${CMAKE_SOURCE_DIR}/third_party/filament)
if(NOT EXISTS ${FILAMENT_DIR}/include/filament/Engine.h)
    message(FATAL_ERROR "Filament not found in ${FILAMENT_DIR}. Run scripts/setup_windows.ps1 first.")
endif()

# Filament ships /MD (release) and /MDd (debug) builds of every library.
set(_fil_release ${FILAMENT_DIR}/lib/x86_64/md)
set(_fil_debug   ${FILAMENT_DIR}/lib/x86_64/mdd)

# Only the libraries we use, in dependency order.
set(FILAMENT_LIBS
    gltfio_core uberarchive uberzlib dracodec meshoptimizer ktxreader basis_transcoder stb image mikktspace
    camutils
    filament backend bluegl bluevk vkshaders filabridge filaflat
    ibl geometry utils smol-v abseil zstd perfetto shaders)

add_library(filament_sdk INTERFACE)
target_include_directories(filament_sdk SYSTEM INTERFACE ${FILAMENT_DIR}/include)
foreach(lib ${FILAMENT_LIBS})
    if(EXISTS ${_fil_release}/${lib}.lib)
        target_link_libraries(filament_sdk INTERFACE
            $<IF:$<CONFIG:Debug>,${_fil_debug}/${lib}.lib,${_fil_release}/${lib}.lib>)
    endif()
endforeach()
if(WIN32)
    target_link_libraries(filament_sdk INTERFACE opengl32 gdi32 user32 shlwapi)
endif()

find_program(FILAMENT_MATC matc PATHS ${FILAMENT_DIR}/bin NO_DEFAULT_PATH REQUIRED)

# cricket_compile_materials(<target> <mat files...>)
# Compiles each .mat to <exe dir>/assets/materials/<name>.filamat.
function(cricket_compile_materials target)
    set(out_dir ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets/materials)
    set(outputs)
    foreach(mat ${ARGN})
        get_filename_component(name ${mat} NAME_WE)
        set(out ${out_dir}/${name}.filamat)
        add_custom_command(
            OUTPUT ${out}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${out_dir}
            COMMAND ${FILAMENT_MATC} -a opengl -a vulkan -p desktop -o ${out} ${mat}
            DEPENDS ${mat}
            COMMENT "matc ${name}.mat"
            VERBATIM)
        list(APPEND outputs ${out})
    endforeach()
    add_custom_target(${target}_materials DEPENDS ${outputs})
    add_dependencies(${target} ${target}_materials)
endfunction()
