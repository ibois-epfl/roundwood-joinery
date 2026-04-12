################################
# Dynamic library of RoundwoodJoinery
################################

file(GLOB_RECURSE SOURCES_LIB
    "${CMAKE_CURRENT_SOURCE_DIR}/RoundwoodJoinery.hh"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
    )
list(REMOVE_ITEM SOURCES_LIB
    "${CMAKE_CURRENT_SOURCE_DIR}/roundwood_joinery_binding.cc"
)

add_library(${SHARED_LIB_NAME} SHARED ${SOURCES_LIB})

if (WIN32)
    set_target_properties(${SHARED_LIB_NAME} PROPERTIES
        WINDOWS_EXPORT_ALL_SYMBOLS TRUE
        )
endif()

set_target_properties(${SHARED_LIB_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/src/RoundwoodJoinery/Debug
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/src/RoundwoodJoinery/Debug
    LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/src/RoundwoodJoinery/Debug
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/src/RoundwoodJoinery/Release
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/src/RoundwoodJoinery/Release
    LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/src/RoundwoodJoinery/Release
)

target_include_directories(${SHARED_LIB_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

add_custom_command(
    TARGET ${SHARED_LIB_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:${SHARED_LIB_NAME}>
        ${CMAKE_CURRENT_SOURCE_DIR}/../python_package/roundwood_joinery/
)