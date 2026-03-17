#include "../../RoundwoodJoinery/RoundwoodJoinery.hh"

int main()
{
    RoundwoodJoinery::PointCloud::PointCloud pointCloud = RoundwoodJoinery::PointCloud::PointCloud();
    if (!pointCloud.LoadFromFile("../../../test_files/ply/cleaned_trunc_00094.ply"))
    {
        std::cerr << "Failed to load point cloud from file." << std::endl;
        return 1;
    }

    Eigen::Vector3d normal(0.0, 0.0, 1.0);
    Eigen::Vector3d corner1(650.0, -490.0, 350.0);
    Eigen::Vector3d corner2(650.0, -290.0, 350.0);
    Eigen::Vector3d corner3(750.0, -290.0, 350.0);
    Eigen::Vector3d corner4(750.0, -490.0, 350.0);
    std::vector<Eigen::Vector3d> corners = {corner1, corner2, corner3, corner4};
    double area1 = 20000.0;

    RoundwoodJoinery::Joinery::JointFace face1(normal, corners, area1);
    std::vector<Eigen::Vector3d> projectedPoints = face1.ProjectPointsOntoFace(pointCloud);
    if(!(projectedPoints.size() == 4589))
    {
        std::cerr << "Test failed: Expected 4589 projected points, got " << projectedPoints.size() << std::endl;
        return 1;
    }
    return 0;
}