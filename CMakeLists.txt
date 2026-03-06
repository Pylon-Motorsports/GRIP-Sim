cmake_minimum_required(VERSION 3.25)
project(GRIP-Sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(cmake/SpvCompile.cmake)
include(FetchContent)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.1
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(glm)

find_package(SDL2 QUIET)
if(NOT SDL2_FOUND)
    message(STATUS "SDL2 not found via find_package; fetching via FetchContent")
    FetchContent_Declare(
        SDL2
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG        release-2.30.9
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(SDL2)
endif()

find_package(Vulkan REQUIRED)

# ---------------------------------------------------------------------------
# Shaders (GLSL -> SPIR-V)
# ---------------------------------------------------------------------------

compile_shaders(Shaders
    OUTPUT_DIR ${CMAKE_BINARY_DIR}/shaders
    SOURCES
        shaders/scene.vert
        shaders/scene.frag
        shaders/sky.vert
        shaders/sky.frag
)

# ---------------------------------------------------------------------------
# Main executable
# ---------------------------------------------------------------------------

set(SOURCES
    src/main.cpp
    src/VulkanContext.cpp
    src/Renderer.cpp
    src/VehiclePhysics.cpp
)

add_executable(GRIP-Sim ${SOURCES})

target_include_directories(GRIP-Sim PRIVATE src)
target_link_libraries(GRIP-Sim PRIVATE
    Vulkan::Vulkan
    SDL2::SDL2
    SDL2::SDL2main
    glm::glm
)

add_dependencies(GRIP-Sim Shaders)

# Copy compiled shaders next to the executable
add_custom_command(TARGET GRIP-Sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:GRIP-Sim>/shaders
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_BINARY_DIR}/shaders
        $<TARGET_FILE_DIR:GRIP-Sim>/shaders
    COMMENT "Copying SPIR-V shaders to output directory"
)

# Copy SDL2 DLL next to the executable (Windows)
if(WIN32)
    add_custom_command(TARGET GRIP-Sim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL2::SDL2>
            $<TARGET_FILE_DIR:GRIP-Sim>
        COMMENT "Copying SDL2 DLL to output directory"
    )
endif()

# ---------------------------------------------------------------------------
# Tests (no Vulkan/SDL dependency — pure physics)
# ---------------------------------------------------------------------------

add_executable(VehicleTest
    tests/vehicle_test.cpp
    src/VehiclePhysics.cpp
)
target_include_directories(VehicleTest PRIVATE src)
target_link_libraries(VehicleTest PRIVATE glm::glm)
