#include "../../RoundwoodJoinery/RoundwoodJoinery.hh"
#include <iostream>

int main()
{
    RoundwoodJoinery::PointCloud::PointCloud pointCloud = RoundwoodJoinery::PointCloud::PointCloud();
    if (!pointCloud.LoadFromFile("../../../test_files/ply/cleaned_trunc_00094.ply"))
    {
        std::cerr << "Failed to load point cloud from file." << std::endl;
        return 1;
    }

    double alpha = 100.0;
    double offset = 0.01;
    std::vector<Eigen::Vector3d> skeleton = RoundwoodJoinery::Utils::ComputePointCloudSkeleton(pointCloud, alpha, offset);

    if (skeleton.size() != 177)
    {
        std::cerr << "Unexpected skeleton size: " << skeleton.size() << std::endl;
        return 1;
    }
    return 0;
}