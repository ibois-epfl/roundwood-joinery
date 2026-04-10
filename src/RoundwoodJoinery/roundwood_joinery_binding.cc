#include "nanobind/nanobind.h"
#include "nanobind/eigen/dense.h"
#include "nanobind/stl/vector.h"
#include "nanobind/stl/pair.h"
#include "nanobind/stl/shared_ptr.h"
#include "RoundwoodJoinery.hh"

namespace nb = nanobind;

NB_MODULE(roundwoodJoineryBindings, m) 
{
    m.attr("PI") = RoundwoodJoinery::PI;

    nb::class_<RoundwoodJoinery::PointCloud::PointCloud>(m, "PointCloud")
        .def(nb::init<>(), "Default constructor for PointCloud")
        .def(nb::init<std::vector<Eigen::Vector3d>>(), "Constructor for PointCloud with given points", nb::arg("points"))
        .def("load_from_file", &RoundwoodJoinery::PointCloud::PointCloud::LoadFromFile, "Load point cloud data from a file", nb::arg("filename"))
        .def("get_points", &RoundwoodJoinery::PointCloud::PointCloud::GetPoints)
        .def("get_1_pcnt_points", &RoundwoodJoinery::PointCloud::PointCloud::Get1PcntPoints);


    nb::class_<RoundwoodJoinery::Joinery::JointFace>(m, "JointFace")
        .def(nb::init<Eigen::Vector3d, 
                      std::vector<Eigen::Vector3d>, 
                      double>(), 
                      "Constructor for JointFace with normal, corners, and target area",
                      nb::arg("normal"), 
                      nb::arg("corners"), 
                      nb::arg("targetArea") = 0.0)
        .def("project_points_onto_face", &RoundwoodJoinery::Joinery::JointFace::ProjectPointsOntoFace,
             "Project points from the beam's point cloud onto the joint face and return the projected points that are within the face outline",
             nb::arg("pointCloud"))
        .def("get_normal", &RoundwoodJoinery::Joinery::JointFace::GetNormal)
        .def("get_corners", &RoundwoodJoinery::Joinery::JointFace::GetCorners)
        .def("get_center", &RoundwoodJoinery::Joinery::JointFace::GetCenter)
        .def("get_target_area", &RoundwoodJoinery::Joinery::JointFace::GetTargetArea)
        .def("get_current_area", &RoundwoodJoinery::Joinery::JointFace::GetCurrentArea)
        .def("compute_current_area", &RoundwoodJoinery::Joinery::JointFace::ComputeCurrentArea,
             "Compute the current area of the joint face based on the projected points from the beam's point cloud",
             nb::arg("beamPointCloud"),
             nb::arg("alpha") = 500.0)
        .def("get_current_outline", &RoundwoodJoinery::Joinery::JointFace::GetCurrentOutline, 
             "Get the current outline of the joint face based on the projected points from the beam's point cloud", 
             nb::arg("beamPointCloud"), 
             nb::arg("alpha") = 500.0);


    nb::class_<RoundwoodJoinery::Joinery::Joint>(m, "Joint")
        .def(nb::init<std::vector<std::shared_ptr<RoundwoodJoinery::Joinery::JointFace>>>(), 
                      "Constructor for Joint with given faces",
                      nb::arg("faces"))
        .def("get_faces", &RoundwoodJoinery::Joinery::Joint::GetFaces)
        .def("get_center", &RoundwoodJoinery::Joinery::Joint::GetCenter)
        .def("get_num_faces", &RoundwoodJoinery::Joinery::Joint::GetNumFaces)
        .def("set_closest_point_on_skeleton", &RoundwoodJoinery::Joinery::Joint::SetClosestPointOnSkeleton, 
                                          "Set the closest point on the skeleton for this joint", 
                                          nb::arg("point"))
        .def("get_closest_point_on_skeleton", &RoundwoodJoinery::Joinery::Joint::GetClosestPointOnSkeleton);


    nb::class_<RoundwoodJoinery::Joinery::JointGroup>(m, "JointGroup")
        .def(nb::init<std::vector<std::shared_ptr<RoundwoodJoinery::Joinery::Joint>>>(), 
                      "Constructor for JointGroup with given joints",
                      nb::arg("joints"))
        .def("get_joints", &RoundwoodJoinery::Joinery::JointGroup::GetJoints)
        .def("set_degree_of_freedom", &RoundwoodJoinery::Joinery::JointGroup::SetDegreeOfFreedom, 
             "Set the degree of freedom for this joint group", 
             nb::arg("degreeOfFreedom"))
        .def("get_degree_of_freedom", &RoundwoodJoinery::Joinery::JointGroup::GetDegreeOfFreedom, 
             "Get the degree of freedom for this joint group")
        .def("get_centroid", &RoundwoodJoinery::Joinery::JointGroup::GetCentroid, 
             "Get the centroid of this joint group, computed as the average of the centers of its joints")
        .def("apply_transformation", &RoundwoodJoinery::Joinery::JointGroup::ApplyTransformation,
             "Apply a 4x4 transformation to this joint group, which will be applied to all its joints and their faces",
             nb::arg("transformation"));

    nb::class_<RoundwoodJoinery::Beam::Beam>(m, "Beam")
        .def(nb::init<double, 
                      std::vector<RoundwoodJoinery::Joinery::JointGroup>, 
                      std::vector<Eigen::Vector3d>, 
                      RoundwoodJoinery::PointCloud::PointCloud>(), 
                      "Constructor for Beam with reference diameter, joints, skeleton, and point cloud", 
                      nb::arg("referenceDiameter"), 
                      nb::arg("jointsByGroup"), 
                      nb::arg("skeleton"), 
                      nb::arg("pointCloud"))
        .def("get_reference_diameter", &RoundwoodJoinery::Beam::Beam::GetReferenceDiameter)
        .def("get_joints_by_group", &RoundwoodJoinery::Beam::Beam::GetJointGroups)
        .def("get_skeleton", &RoundwoodJoinery::Beam::Beam::GetSkeleton)
        .def("get_point_cloud", &RoundwoodJoinery::Beam::Beam::GetPointCloud)
        .def("find_joint_closest_points_on_skeleton", &RoundwoodJoinery::Beam::Beam::FindJointClosestPointsOnSkeleton, 
                                                 "For each joint of the beam, find the closest point on the beam skeleton and set it as the closest point on skeleton for the joint")
        .def("compute_one_iteration_of_joint_face_translations_for_optimisation", &RoundwoodJoinery::Beam::Beam::ComputeOneIterationOfJointFaceTranslationsForOptimisation, 
                                                                          "Compute one iteration of joint face translations for optimization based on the current state of the beam and its joints")
        .def("compute_joint_group_optimisation", &RoundwoodJoinery::Beam::Beam::ComputeJointGroupOptimisation, 
             "Compute an optimization of the joint group transformations to better align the joint faces with the beam's point cloud skeleton",
             nb::arg("maxIterations") = 10, 
             nb::arg("minRelativeTranslationRMSE") = 1.0);

        nb::module_ u = m.def_submodule("Utils", "Utility functions for Roundwood Joinery");

        u.def("compute_point_cloud_skeleton", &RoundwoodJoinery::Utils::ComputePointCloudSkeleton, 
            "Compute the skeleton of a given point cloud using the alpha shape algorithm", 
            nb::arg("pointCloud"), 
            nb::arg("alpha"), 
            nb::arg("offset"));

        u.def("find_height_of_triangle", &RoundwoodJoinery::Utils::FindHeightOfTriangle, 
            "Compute the height of a triangle formed by a test point and a base defined by two points", 
            nb::arg("testPoint"), 
            nb::arg("baseStart"), 
            nb::arg("baseEnd"));

        u.def("compute_2d_alpha_shape", &RoundwoodJoinery::Utils::Compute2DAlphaShape, 
            "Compute the 2D alpha shape of a set of points that all lie on the same plane using CGAL", 
            nb::arg("points"), 
            nb::arg("alpha"), 
            nb::arg("normal"));

        u.def("save_point_cloud_to_ply", &RoundwoodJoinery::Utils::SavePointCloudToPLY, 
            "Save a point cloud to a PLY file", 
            nb::arg("points"), 
            nb::arg("filename"));
            
        u.def("compute_approximating_transformation", &RoundwoodJoinery::Utils::ComputeApproximatingTransformation,
            "Compute an approximating transformation matrix based on a set of anchor points and their corresponding translations",
            nb::arg("anchorPointsAndTranslations"));
}