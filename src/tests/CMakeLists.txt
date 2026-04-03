# Iterate over each test file and create the corresponding executable, link libraries, and add tests
set(SHARED_LIB_COPIED FALSE)
foreach(TEST_FILE ${UNIT_TEST_FILES})
    message(STATUS "Processing test file: ${TEST_FILE}")
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    message(STATUS "Test name: ${TEST_NAME}")
    add_executable(${TEST_NAME} ${TEST_FILE})
    if(WIN32)
        set_target_properties(${TEST_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/src/tests/Debug
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/src/tests/Release
        )
    endif()
    target_link_libraries(${TEST_NAME} PRIVATE ${SHARED_LIB_NAME})
    if(WIN32)
        add_test(NAME ${TEST_NAME}
            COMMAND $<TARGET_FILE:${TEST_NAME}>)
    else()
        add_test(NAME ${TEST_NAME}
            COMMAND ${CMAKE_BINARY_DIR}/src/tests/${TEST_NAME})
    endif()
    add_dependencies(${TEST_NAME} ${SHARED_LIB_NAME})

    # Copy the shared library to the test executable directory after building the test executables
    if(NOT SHARED_LIB_COPIED) # Ensure we only set up the copy command once
        if(WIN32)
            set(SHARED_LIB_FILE ${CMAKE_BINARY_DIR}/src/RoundwoodJoinery/$<CONFIG>/roundwoodJoinery.dll)
        elseif(APPLE)
            set(SHARED_LIB_FILE ${CMAKE_BINARY_DIR}/src/RoundwoodJoinery/$<CONFIG>/libroundwoodJoinery.dylib)
        else()
            set(SHARED_LIB_FILE ${CMAKE_BINARY_DIR}/src/RoundwoodJoinery/$<CONFIG>/libroundwoodJoinery.so)
        endif()
        set(SHARED_LIB_COPIED TRUE)
        add_custom_command(TARGET ${TEST_NAME} POST_BUILD
                            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                                ${SHARED_LIB_FILE}
                                $<TARGET_FILE_DIR:${TEST_NAME}>
                            )
    endif()
endforeach()
