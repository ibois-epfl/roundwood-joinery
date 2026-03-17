################################
# Dynamic library of RoundwoodJoinery
################################

file(GLOB_RECURSE SOURCES_LIB
    "${CMAKE_CURRENT_SOURCE_DIR}/RoundwoodJoinery.hh"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")

add_library(${SHARED_LIB_NAME} ${SOURCES_LIB})