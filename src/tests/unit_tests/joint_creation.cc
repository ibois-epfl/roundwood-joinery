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
    double targetArea = 15000.0;

    std::shared_ptr<RoundwoodJoinery::Joinery::JointFace> face1 = std::make_shared<RoundwoodJoinery::Joinery::JointFace>(normal, corners, targetArea);

    // Create a Joint instance with the created face
    std::vector<std::shared_ptr<RoundwoodJoinery::Joinery::JointFace>> faces = {face1};
    RoundwoodJoinery::Joinery::Joint joint(faces);

    if (joint.GetNumFaces() != 1)
    {
        std::cerr << "Test failed: Expected 1 face, got " << joint.GetNumFaces() << std::endl;
        return 1;
    }

    if (joint.GetFaces()[0]->GetTargetArea() != targetArea)
    {
        std::cerr << "Test failed: Expected target area " << targetArea << ", got " << joint.GetFaces()[0]->GetTargetArea() << std::endl;
        return 1;
    }

    if (joint.GetFaces()[0]->GetNormal() - normal != Eigen::Vector3d::Zero())
    {
        std::cerr << "Test failed: Expected normal " << normal.transpose() << ", got " << joint.GetFaces()[0]->GetNormal().transpose() << std::endl;
        return 1;
    }
    if (joint.GetFaces()[0]->GetCenter() - center != Eigen::Vector3d::Zero())
    {
        std::cerr << "Test failed: Expected center " << center.transpose() << ", got " << joint.GetFaces()[0]->GetCenter().transpose() << std::endl;
        return 1;
    }

    RoundwoodJoinery::PointCloud::PointCloud pointCloud = RoundwoodJoinery::PointCloud::PointCloud();
    if (!pointCloud.LoadFromFile("../../../test_files/ply/cleaned_trunc_00094.ply"))
    {
        std::cerr << "Failed to load point cloud from file." << std::endl;
        return 1;
    }
    if( joint.GetFaces()[0]->ComputeCurrentArea(pointCloud) < 11900.0 || joint.GetFaces()[0]->ComputeCurrentArea(pointCloud) > 11910.0 )
    {
        std::cerr << "Test failed: Expected current area between 11900.0 and 11910.0, got " << joint.GetFaces()[0]->ComputeCurrentArea(pointCloud) << std::endl;
        return 1;
    }
    return 0;
}