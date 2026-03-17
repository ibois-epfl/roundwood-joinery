# Iterate over each test file and create the corresponding executable, link libraries, and add tests
foreach(TEST_FILE ${UNIT_TEST_FILES})
    message(STATUS "Processing test file: ${TEST_FILE}")
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    message(STATUS "Test name: ${TEST_NAME}")
    add_executable(${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${TEST_NAME} ${SHARED_LIB_NAME})
    add_test(NAME ${TEST_NAME}
             COMMAND ${CMAKE_BINARY_DIR}/src/tests/Debug/${TEST_NAME})
    add_dependencies(${TEST_NAME} ${SHARED_LIB_NAME})
endforeach()
