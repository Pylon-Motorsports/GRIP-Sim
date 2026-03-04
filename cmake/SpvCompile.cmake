function(compile_shaders TARGET_NAME)
    cmake_parse_arguments(ARG "" "OUTPUT_DIR" "SOURCES" ${ARGN})

    find_program(GLSLC glslc HINTS $ENV{VULKAN_SDK}/bin)
    if(NOT GLSLC)
        message(FATAL_ERROR "glslc not found. Ensure the Vulkan SDK is installed and VULKAN_SDK is set.")
    endif()

    set(SPV_OUTPUTS)
    foreach(GLSL ${ARG_SOURCES})
        get_filename_component(FNAME ${GLSL} NAME)
        set(SPV "${ARG_OUTPUT_DIR}/${FNAME}.spv")
        add_custom_command(
            OUTPUT  ${SPV}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${ARG_OUTPUT_DIR}
            COMMAND ${GLSLC} ${CMAKE_SOURCE_DIR}/${GLSL} -o ${SPV}
            DEPENDS ${CMAKE_SOURCE_DIR}/${GLSL}
            COMMENT "Compiling shader: ${GLSL} -> ${FNAME}.spv"
            VERBATIM
        )
        list(APPEND SPV_OUTPUTS ${SPV})
    endforeach()

    add_custom_target(${TARGET_NAME} DEPENDS ${SPV_OUTPUTS})
endfunction()
