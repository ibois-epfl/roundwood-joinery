#include "../../RoundwoodJoinery/RoundwoodJoinery.hh"
#include <iostream>

int main()
{
    // Create a JointFace instance
    Eigen::Vector3d normal(0.0, 0.0, 1.0);
    Eigen::Vector3d center(700.0, -390.0, 350.0);
    Eigen::Vector3d corner1(650.0, -490.0, 350.0);
    Eigen::Vector3d corner2(650.0, -290.0, 350.0);
    Eigen::Vector3d corner3(750.0, -290.0, 350.0);
    Eigen::Vector3d corner4(750.0, -490.0, 350.0);
    std::vector<Eigen::Vector3d> corners = {corner1, corner2, corner3, corner4};
    double targetArea = 12000.0;

    RoundwoodJoinery::Joinery::JointFace face1(normal, corners, targetArea);

    // Create a Joint instance with the created face
    std::vector<RoundwoodJoinery::Joinery::JointFace> faces = {face1};
    RoundwoodJoinery::Joinery::Joint joint(faces);
    std::vector<std::shared_ptr<RoundwoodJoinery::Joinery::Joint>> jointVector = {std::make_shared<RoundwoodJoinery::Joinery::Joint>(joint)};

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
    RoundwoodJoinery::Beam::Beam beam(referenceDiameter, {jointVector}, skeleton, pointCloud);

    double retrievedReferenceDiameter = beam.GetReferenceDiameter();

    if (retrievedReferenceDiameter != referenceDiameter)
    {
        std::cout << "Beam reference diameter incorrect: " << retrievedReferenceDiameter << std::endl;
        return 1;
    }

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> translations = beam.ComputeOneIterationOfJointFaceTranslationsForOptimisation();
    for (const auto& [anchor, translation] : translations)
    {
        std::cout << "Anchor: " << anchor.transpose() << ", Translation: " << translation.transpose() << std::endl;
    }
    return 0;
}