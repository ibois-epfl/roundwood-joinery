#include "nanobind/nanobind.h"
#include "nanobind/eigen/dense.h"
#include "nanobind/stl/vector.h"
#include "RoundwoodJoinery.hh"

namespace nb = nanobind;

NB_MODULE(roundwoodJoineryBindings, m) 
{
    m.attr("M_PI") = RoundwoodJoinery::M_PI;

    nanobind::class_<RoundwoodJoinery::PointCloud::PointCloud>(m, "PointCloud")
        .def(nb::init<>(), "Default constructor for PointCloud")
        .def(nb::init<std::vector<Eigen::Vector3d>>(), "Constructor for PointCloud with given points")
        .def("LoadFromFile", &RoundwoodJoinery::PointCloud::PointCloud::LoadFromFile)
        .def("GetPoints", &RoundwoodJoinery::PointCloud::PointCloud::GetPoints)
        .def("Get1PcntPoints", &RoundwoodJoinery::PointCloud::PointCloud::Get1PcntPoints);

    nanobind::class_<RoundwoodJoinery::Joinery::JointFace>(m, "JointFace")
        .def(nb::init<Eigen::Vector3d, std::vector<Eigen::Vector3d>, double>(), "Constructor for JointFace with normal, corners, and target area")
        .def("ProjectPointsOntoFace", &RoundwoodJoinery::Joinery::JointFace::ProjectPointsOntoFace)
        .def("GetNormal", &RoundwoodJoinery::Joinery::JointFace::GetNormal)
        .def("GetCenter", &RoundwoodJoinery::Joinery::JointFace::GetCenter)
        .def("GetTargetArea", &RoundwoodJoinery::Joinery::JointFace::GetTargetArea)
        .def("GetCurrentArea", &RoundwoodJoinery::Joinery::JointFace::GetCurrentArea);

        
}
