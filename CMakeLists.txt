cmake_minimum_required (VERSION 3.18)
project (roundwood_joinery VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

add_definitions(-DCGAL_DISABLE_GMP)

set(RUN_TESTS TRUE) #For now we will always run tests after building...

set (SHARED_LIB_NAME "roundwoodJoinery")
add_subdirectory(src/RoundwoodJoinery)
set (BINDINGS_MODULE_NAME "roundwoodJoineryBindings")

execute_process(
    COMMAND git submodule update --init --recursive
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE GIT_SUBMOD_RESULT
    )
if(NOT GIT_SUBMOD_RESULT EQUAL "0")
    message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
endif()

# to avoid "fatal error C1128: number of sections exceeded object file format limit: compile with /bigobj" on MSVC
if (MSVC)
    target_compile_options(${SHARED_LIB_NAME} PRIVATE /bigobj)
endif()

#############
# 3rd parties
#############

# Eigen
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/3rd_party/eigen)
target_include_directories(${SHARED_LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/3rd_party/eigen)

# Boost
file(GLOB BOOST_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src/3rd_party/boost/libs/*/include")
foreach(dir ${BOOST_INCLUDE_DIRS})
    message("Adding Boost include dir: ${dir}")
    include_directories(${dir})
    target_include_directories(${SHARED_LIB_NAME} PUBLIC ${dir})
endforeach()

# CGAL
file(GLOB CGAL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src/3rd_party/cgal/*/include")
foreach(dir ${CGAL_INCLUDE_DIRS})
    message("Adding CGAL include dir: ${dir}")
    include_directories(${dir})
    target_include_directories(${SHARED_LIB_NAME} PUBLIC ${dir})
endforeach()


# nanobind
if (CMAKE_VERSION VERSION_LESS 3.18)
  set(DEV_MODULE Development)
else()
  set(DEV_MODULE Development.Module)
endif()
find_package(Python 3.9 COMPONENTS Interpreter ${DEV_MODULE} REQUIRED)
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/3rd_party/nanobind)
nanobind_add_module(${BINDINGS_MODULE_NAME} ${PROJECT_SOURCE_DIR}/src/RoundwoodJoinery/roundwood_joinery_binding.cc)
target_link_libraries(${BINDINGS_MODULE_NAME} PRIVATE ${SHARED_LIB_NAME})


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

if(WIN32)
    add_custom_target(run_tests
        COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIG> --output-on-failure
        DEPENDS ${TEST_EXECUTABLES}
        COMMENT "Running all tests")
else()
    add_custom_target(run_tests
        COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
        DEPENDS ${TEST_EXECUTABLES}
        COMMENT "Running all tests")
        
        add_custom_target(check ALL
        DEPENDS run_tests)
endif()

add_custom_command(
    TARGET ${BINDINGS_MODULE_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:${BINDINGS_MODULE_NAME}>
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tests/py_binding_test/
)