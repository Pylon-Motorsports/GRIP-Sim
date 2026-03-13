# SnapshotBuild.cmake — copies exe + shaders + SDL2 DLL into a timestamped dir.
# Usage: cmake -P SnapshotBuild.cmake <exe_path> <shaders_dir> <sdl2_dll> <output_root>

set(EXE_PATH    "${CMAKE_ARGV3}")
set(SHADERS_DIR "${CMAKE_ARGV4}")
set(SDL2_DLL    "${CMAKE_ARGV5}")
set(OUTPUT_ROOT "${CMAKE_ARGV6}")

string(TIMESTAMP NOW "%Y-%m-%d_%H%M%S")
set(SNAP_DIR "${OUTPUT_ROOT}/${NOW}")

file(MAKE_DIRECTORY "${SNAP_DIR}")
file(MAKE_DIRECTORY "${SNAP_DIR}/shaders")

# Copy exe
file(COPY "${EXE_PATH}" DESTINATION "${SNAP_DIR}")

# Copy shaders
file(GLOB SPV_FILES "${SHADERS_DIR}/*.spv")
foreach(f ${SPV_FILES})
    file(COPY "${f}" DESTINATION "${SNAP_DIR}/shaders")
endforeach()

# Copy SDL2 DLL
if(EXISTS "${SDL2_DLL}")
    file(COPY "${SDL2_DLL}" DESTINATION "${SNAP_DIR}")
endif()

message(STATUS "Build snapshot: ${SNAP_DIR}")
