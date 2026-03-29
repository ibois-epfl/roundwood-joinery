#include "../../RoundwoodJoinery/RoundwoodJoinery.hh"
#include <cmath>

int main()
{
    RoundwoodJoinery::PointCloud::PointCloud pointCloud = RoundwoodJoinery::PointCloud::PointCloud();
    if (!pointCloud.LoadFromFile("../../../test_files/ply/cleaned_trunc_00094.ply"))
    {
        std::cerr << "Failed to load point cloud from file." << std::endl;
        return 1;
    }

    Eigen::Vector3d normal(std::cos(std::acos(-1) / 4), 0.0, std::sin(std::acos(-1) / 4));
    Eigen::Vector3d corner1(680.0, -490.0, 420.0);
    Eigen::Vector3d corner2(680.0, -290.0, 420.0);
    Eigen::Vector3d corner3(750.0, -290.0, 350.0);
    Eigen::Vector3d corner4(750.0, -490.0, 350.0);
    std::vector<Eigen::Vector3d> corners = {corner1, corner2, corner3, corner4};
    double targetArea = 15000.0;

    RoundwoodJoinery::Joinery::JointFace face1(normal, corners, targetArea);
    std::vector<Eigen::Vector3d> projectedPoints = face1.ProjectPointsOntoFace(pointCloud);
    if(!(projectedPoints.size() == 1783 ))
    {
        std::cerr << "Test failed: Expected 1783 projected points, got " << projectedPoints.size() << std::endl;
        return 1;
    }
    return 0;
}