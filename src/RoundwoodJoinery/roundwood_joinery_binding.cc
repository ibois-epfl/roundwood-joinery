#include "nanobind/nanobind.h"
#include "nanobind/eigen/dense.h"
#include "nanobind/stl/vector.h"
#include "nanobind/stl/shared_ptr.h"
#include "RoundwoodJoinery.hh"

namespace nb = nanobind;

NB_MODULE(roundwoodJoineryBindings, m) 
{
    m.attr("PI") = RoundwoodJoinery::PI;

    nb::class_<RoundwoodJoinery::PointCloud::PointCloud>(m, "PointCloud")
        .def(nb::init<>(), "Default constructor for PointCloud")
        .def(nb::init<std::vector<Eigen::Vector3d>>(), "Constructor for PointCloud with given points", nb::arg("points"))
        .def("LoadFromFile", &RoundwoodJoinery::PointCloud::PointCloud::LoadFromFile, "Load point cloud data from a file", nb::arg("filename"))
        .def("GetPoints", &RoundwoodJoinery::PointCloud::PointCloud::GetPoints)
        .def("Get1PcntPoints", &RoundwoodJoinery::PointCloud::PointCloud::Get1PcntPoints);


    nb::class_<RoundwoodJoinery::Joinery::JointFace>(m, "JointFace")
        .def(nb::init<Eigen::Vector3d, 
                      std::vector<Eigen::Vector3d>, 
                      double>(), 
                      "Constructor for JointFace with normal, corners, and target area",
                      nb::arg("normal"), 
                      nb::arg("corners"), 
                      nb::arg("targetArea") = 0.0)
        .def("ProjectPointsOntoFace", &RoundwoodJoinery::Joinery::JointFace::ProjectPointsOntoFace)
        .def("GetNormal", &RoundwoodJoinery::Joinery::JointFace::GetNormal)
        .def("GetCenter", &RoundwoodJoinery::Joinery::JointFace::GetCenter)
        .def("GetTargetArea", &RoundwoodJoinery::Joinery::JointFace::GetTargetArea)
        .def("GetCurrentArea", &RoundwoodJoinery::Joinery::JointFace::GetCurrentArea)
        .def("ComputeCurrentArea", &RoundwoodJoinery::Joinery::JointFace::ComputeCurrentArea);


    nb::class_<RoundwoodJoinery::Joinery::Joint>(m, "Joint")
        .def(nb::init<std::vector<RoundwoodJoinery::Joinery::JointFace>>(), 
                      "Constructor for Joint with given faces",
                      nb::arg("faces"))
        .def("GetFaces", &RoundwoodJoinery::Joinery::Joint::GetFaces)
        .def("GetCenter", &RoundwoodJoinery::Joinery::Joint::GetCenter)
        .def("GetNumFaces", &RoundwoodJoinery::Joinery::Joint::GetNumFaces)
        .def("SetClosestPointOnSkeleton", &RoundwoodJoinery::Joinery::Joint::SetClosestPointOnSkeleton, 
                                          "Set the closest point on the skeleton for this joint", 
                                          nb::arg("point"))
        .def("GetClosestPointOnSkeleton", &RoundwoodJoinery::Joinery::Joint::GetClosestPointOnSkeleton);


    nb::class_<RoundwoodJoinery::Beam::Beam>(m, "Beam")
        .def(nb::init<double, 
                      std::vector<std::shared_ptr<RoundwoodJoinery::Joinery::Joint>>, 
                      std::vector<Eigen::Vector3d>, 
                      RoundwoodJoinery::PointCloud::PointCloud>(), 
                      "Constructor for Beam with reference diameter, joints, skeleton, and point cloud", 
                      nb::arg("referenceDiameter"), 
                      nb::arg("joints"), 
                      nb::arg("skeleton"), 
                      nb::arg("pointCloud"))
        .def("GetReferenceDiameter", &RoundwoodJoinery::Beam::Beam::GetReferenceDiameter)
        .def("GetJoints", &RoundwoodJoinery::Beam::Beam::GetJoints)
        .def("GetSkeleton", &RoundwoodJoinery::Beam::Beam::GetSkeleton)
        .def("GetPointCloud", &RoundwoodJoinery::Beam::Beam::GetPointCloud)
        .def("FindJointClosestPointsOnSkeleton", &RoundwoodJoinery::Beam::Beam::FindJointClosestPointsOnSkeleton, 
                                                 "For each joint of the beam, find the closest point on the beam skeleton and set it as the closest point on skeleton for the joint");
}   
