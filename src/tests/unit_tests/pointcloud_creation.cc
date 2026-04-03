#include "../../RoundwoodJoinery/RoundwoodJoinery.hh"
#include <iostream>

int main()
{
    RoundwoodJoinery::PointCloud::PointCloud pointCloud1 = RoundwoodJoinery::PointCloud::PointCloud();
    RoundwoodJoinery::PointCloud::PointCloud pointCloud2 = RoundwoodJoinery::PointCloud::PointCloud({Eigen::Vector3d(1.0, 2.0, 3.0), 
                                                                                                     Eigen::Vector3d(4.0, 5.0, 6.0)});
    if (!pointCloud1.LoadFromFile("../../../test_files/ply/cleaned_trunc_00094.ply"))
    {
        std::cerr << "Failed to load point cloud from file." << std::endl;
        return 1;
    }

    std::vector<Eigen::Vector3d> points1 = pointCloud1.GetPoints();
    std::vector<Eigen::Vector3d> points2 = pointCloud2.GetPoints();
    if (points1.size() != 631250)
    {
        std::cerr << "Test failed: Expected 631250 points, got " << points1.size() << std::endl;
        return 1;
    }
    if (points2.size() != 2)
    {
        std::cerr << "Test failed: Expected 2 points, got " << points2.size() << std::endl;
        return 1;
    }
    return 0;
}