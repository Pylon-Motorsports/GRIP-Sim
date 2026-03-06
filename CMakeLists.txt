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

# SDL2 — use FetchContent if not already installed via vcpkg/system package
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
    # FetchContent SDL2 exposes SDL2::SDL2 and SDL2::SDL2main targets
endif()

find_package(Vulkan REQUIRED)

# ---------------------------------------------------------------------------
# Shader compilation
# ---------------------------------------------------------------------------

set(SHADER_OUTPUT_DIR ${CMAKE_BINARY_DIR}/shaders)

compile_shaders(
    GRIP_Sim_Shaders
    SOURCES
        shaders/road.vert
        shaders/road.frag
        shaders/sky.vert
        shaders/sky.frag
        shaders/debug_line.vert
        shaders/debug_line.frag
        shaders/vehicle.vert
        shaders/vehicle.frag
    OUTPUT_DIR
        ${SHADER_OUTPUT_DIR}
)

# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------

set(SOURCES
    src/main.cpp
    src/core/Engine.cpp
    src/rendering/VulkanContext.cpp
    src/rendering/VulkanRoadRenderer.cpp
    src/rendering/VulkanPipeline.cpp
    src/rendering/VulkanBuffer.cpp
    src/rendering/VehicleMesh.cpp
    src/rendering/HudOverlay.cpp
    src/rendering/Camera.cpp
    src/road/RoadBuilder.cpp
    src/road/TestStageGenerator.cpp
    src/road/TerrainQuery.cpp
    src/vehicle/SemiRealisticVehicle.cpp
    src/vehicle/MultiBodyVehicle.cpp
    src/vehicle/CubeVehicle.cpp
    src/input/SdlInputProvider.cpp
    src/pacenote/PaceNote.cpp
    src/pacenote/GripPaceNoteGenerator.cpp
    src/pacenote/ConsolePaceNoteReader.cpp
)

if(WIN32)
    list(APPEND SOURCES src/pacenote/WindowsSapiReader.cpp)
endif()

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------

add_executable(${PROJECT_NAME} ${SOURCES})

add_dependencies(${PROJECT_NAME} GRIP_Sim_Shaders)

target_include_directories(${PROJECT_NAME} PRIVATE
    src/
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    SHADER_DIR="${SHADER_OUTPUT_DIR}/"
    GLM_ENABLE_EXPERIMENTAL
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    glm::glm
    SDL2::SDL2
    SDL2::SDL2main
    Vulkan::Vulkan
)

if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE sapi ole32)
endif()

include(cmake/CompilerFlags.cmake)

# ---------------------------------------------------------------------------
# Vehicle physics unit tests (no Vulkan/SDL2 dependency)
# ---------------------------------------------------------------------------

set(PHYSICS_TEST_SOURCES
    tests/vehicle_physics_test.cpp
    src/vehicle/MultiBodyVehicle.cpp
    src/vehicle/CubeVehicle.cpp
    src/road/TerrainQuery.cpp
    src/road/RoadBuilder.cpp
    src/road/TestStageGenerator.cpp
)

add_executable(VehiclePhysicsTest ${PHYSICS_TEST_SOURCES})

target_include_directories(VehiclePhysicsTest PRIVATE src/)
target_compile_definitions(VehiclePhysicsTest PRIVATE GLM_ENABLE_EXPERIMENTAL)
target_compile_features(VehiclePhysicsTest PRIVATE cxx_std_20)
target_link_libraries(VehiclePhysicsTest PRIVATE glm::glm)

enable_testing()
add_test(NAME VehiclePhysics COMMAND VehiclePhysicsTest)
