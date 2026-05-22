#include <iostream>

#include "../../RoundwoodJoinery/RoundwoodJoinery.hh"

int main()
{
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

    std::vector<Eigen::Vector3d> testPoints;
    for (int i = 0; i < skeleton.size()/10; ++i)
    {
        testPoints.push_back(skeleton[i*10]);
    }
    Eigen::Matrix3d rotation = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3,3>(0,0) = rotation;
    transformation.block<3,1>(0,3) = Eigen::Vector3d(1000.0, 500.0, -300.0);

    std::vector<Eigen::Vector3d> transformedTestPoints = {};
    for (const auto& point : testPoints)
    {
        Eigen::Vector4d homogenousPoint(point.x(), point.y(), point.z(), 1.0);
        Eigen::Vector4d transformedPoint = transformation * homogenousPoint;
        transformedTestPoints.push_back(transformedPoint.head<3>());
    }

    Eigen::Matrix4d computedTransformation = RoundwoodJoinery::Utils::ComputeCurveToCurveTransformation(transformedTestPoints, testPoints);

    std::cout << "Computed transformation: " << std::endl << computedTransformation << std::endl;
    std::cout << "Ground truth transformation: " << std::endl << transformation << std::endl;
    if (computedTransformation.isApprox(transformation, 1e-2))
    {
        std::cout << "Test passed: Computed transformation is approximately equal to the ground truth transformation." << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Test failed: Computed transformation is not approximately equal to the ground truth transformation." << std::endl;
        return 1;
    }
}