cmake_minimum_required (VERSION 3.14)
project (roundwood_joinery)

set (SHARED_LIB_NAME "roundwoodJoinery")
add_subdirectory(src/RoundwoodJoinery)

execute_process(
    COMMAND git submodule update --init --recursive
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE GIT_SUBMOD_RESULT
    )
if(NOT GIT_SUBMOD_RESULT EQUAL "0")
    message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
endif()

#############
# 3rd parties
#############

# Eigen
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/3rd_party/eigen)
target_include_directories(${SHARED_LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/3rd_party/eigen)

#############
# Unit tests
#############

# Get all test files in the ./UnitTests/ directory
file(GLOB UNIT_TEST_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/unit_tests/*.cc")

enable_testing()
include(CTest)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/tests)

set(TEST_EXECUTABLES)
foreach(TEST_FILE ${UNIT_TEST_FILES})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    list(APPEND TEST_EXECUTABLES ${TEST_NAME})
endforeach()

add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS ${TEST_EXECUTABLES}) 