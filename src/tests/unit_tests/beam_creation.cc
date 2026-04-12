#include "../../RoundwoodJoinery/RoundwoodJoinery.hh"
#include <iostream>

int main()
{
    // Create a JointFace instance
    Eigen::Vector3d normal(0.0, 0.0, -1.0);
    Eigen::Vector3d center(700.0, -390.0, 350.0);
    Eigen::Vector3d corner1(650.0, -490.0, 350.0);
    Eigen::Vector3d corner2(650.0, -290.0, 350.0);
    Eigen::Vector3d corner3(750.0, -290.0, 350.0);
    Eigen::Vector3d corner4(750.0, -490.0, 350.0);
    std::vector<Eigen::Vector3d> corners = {corner1, corner2, corner3, corner4};
    double targetArea = 10000.0;

    std::shared_ptr<RoundwoodJoinery::Joinery::JointFace> face1 = std::make_shared<RoundwoodJoinery::Joinery::JointFace>(normal, corners, targetArea);

    // Create a Joint instance with the created face
    std::vector<std::shared_ptr<RoundwoodJoinery::Joinery::JointFace>> faces = {face1};
    RoundwoodJoinery::Joinery::Joint joint(faces);
    std::vector<RoundwoodJoinery::Joinery::JointGroup> jointGroups = {RoundwoodJoinery::Joinery::JointGroup({std::make_shared<RoundwoodJoinery::Joinery::Joint>(joint)})};

    // Computing the skeleton of the pointcloud
    RoundwoodJoinery::PointCloud::PointCloud pointCloud = RoundwoodJoinery::PointCloud::PointCloud();

    if (!pointCloud.LoadFromFile("../../../test_files/ply/cleaned_trunc_00094.ply"))
    {
        std::cerr << "Failed to load point cloud from file." << std::endl;
        return 1;
    }

    double alpha = 100.0;
    double offset = 0.01;
    std::vector<Eigen::Vector3d> skeleton = RoundwoodJoinery::Utils::ComputePointCloudSkeleton(pointCloud, alpha, offset);

    double referenceDiameter = 160.0; // corresponds more or less to test data.
    RoundwoodJoinery::Beam::Beam beam(referenceDiameter, jointGroups, skeleton, pointCloud);

    double retrievedReferenceDiameter = beam.GetReferenceDiameter();

    if (retrievedReferenceDiameter != referenceDiameter)
    {
        std::cout << "Beam reference diameter incorrect: " << retrievedReferenceDiameter << std::endl;
        return 1;
    }
    if (beam.GetJointGroups()[0].GetJoints()[0]->GetFaces()[0]->GetNormal() != Eigen::Vector3d(0.0, 0.0, 1.0))
    {
        std::cout << "Beam joint face normal should have been flipped to (0,0,1) but is: " << beam.GetJointGroups()[0].GetJoints()[0]->GetFaces()[0]->GetNormal().transpose() << std::endl;
        return 1;
    }
    if (beam.GetJointGroups().size() != jointGroups.size())
    {
        std::cout << "Beam joint groups incorrect: " << beam.GetJointGroups().size() << std::endl;
        return 1;
    }
    double initialArea = face1->ComputeCurrentArea(pointCloud, 500.0);
    std::cout << "Initial Joint Face area: " << initialArea << std::endl;

    int maxIterations = 100;
    double minRelativeTranslationRMSE = 1.0; // mm
    std::vector<Eigen::Matrix4d> transformations = beam.ComputeJointGroupOptimisation(maxIterations, minRelativeTranslationRMSE);
    for (size_t index = 0; index < transformations.size(); ++index)
    {
        std::cout << "---> Transformation: " << std::endl << transformations[index] << std::endl;
    }
    double finalArea = face1->ComputeCurrentArea(pointCloud, 500.0);
    std::cout << "Final Joint Face area: " << finalArea << std::endl;
    return 0;
}