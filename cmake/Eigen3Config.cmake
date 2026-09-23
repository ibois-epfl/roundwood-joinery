# Because CPD wants Eigen3 and I have Eigen (same but different alias)
if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/../src/3rd_party/eigen"
    )
endif()

set(Eigen3_FOUND TRUE)