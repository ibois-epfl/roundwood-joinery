# Iterate over each test file and create the corresponding executable, link libraries, and add tests
set(DLL_COPIED FALSE)
foreach(TEST_FILE ${UNIT_TEST_FILES})
    message(STATUS "Processing test file: ${TEST_FILE}")
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    message(STATUS "Test name: ${TEST_NAME}")
    add_executable(${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${TEST_NAME} PRIVATE ${SHARED_LIB_NAME})
    add_test(NAME ${TEST_NAME}
         COMMAND ${CMAKE_BINARY_DIR}/src/tests/$<CONFIG>/${TEST_NAME})
    add_dependencies(${TEST_NAME} ${SHARED_LIB_NAME})

    # Copy the DLLs to the test executable directory after building the test executables
    if(NOT DLL_COPIED) # Ensure we only set up the copy command once
        set(DLL_COPIED TRUE)
        add_custom_command(TARGET ${TEST_NAME} POST_BUILD
                            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                                ${CMAKE_BINARY_DIR}/src/RoundwoodJoinery/$<CONFIG>/roundwoodJoinery.dll
                                $<TARGET_FILE_DIR:${TEST_NAME}>
                            )
    endif()

    if(${RUN_TESTS})
        add_custom_command(
                        TARGET ${TEST_NAME} POST_BUILD
                        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                        COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> --verbose
                        COMMENT "Running all tests"
                        )
    endif()
endforeach()
