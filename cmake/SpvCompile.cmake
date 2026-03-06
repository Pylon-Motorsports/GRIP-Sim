find_program(GLSLC glslc HINTS "$ENV{VULKAN_SDK}/Bin")
if(NOT GLSLC)
    message(FATAL_ERROR "glslc not found – install the Vulkan SDK and ensure it is on PATH")
endif()

function(compile_shaders TARGET_NAME)
    cmake_parse_arguments(ARG "" "OUTPUT_DIR" "SOURCES" ${ARGN})
    file(MAKE_DIRECTORY ${ARG_OUTPUT_DIR})
    set(_outputs)
    foreach(src ${ARG_SOURCES})
        get_filename_component(name ${src} NAME)
        set(out ${ARG_OUTPUT_DIR}/${name}.spv)
        add_custom_command(
            OUTPUT  ${out}
            COMMAND ${GLSLC} ${CMAKE_SOURCE_DIR}/${src} -o ${out}
            DEPENDS ${CMAKE_SOURCE_DIR}/${src}
            COMMENT "Compiling ${src} -> ${name}.spv"
        )
        list(APPEND _outputs ${out})
    endforeach()
    add_custom_target(${TARGET_NAME} ALL DEPENDS ${_outputs})
endfunction()
