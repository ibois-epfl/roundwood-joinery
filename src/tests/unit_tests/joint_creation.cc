#include "../../RoundwoodJoinery/RoundwoodJoinery.hh"
#include <iostream>

int main()
{
    // Create a JointFace instance
    Eigen::Vector3d normal1(0.0, 0.0, 1.0);
    Eigen::Vector3d center1(0.0, 0.0, 0.0);
    double area1 = 1.0;

    Eigen::Vector3d normal2(0.0, 1.0, 0.0);
    Eigen::Vector3d center2(0.0, -1.0, 0.0);
    double area2 = 1.0;

    Eigen::Vector3d normal3(0.0, -1.0, 0.0);
    Eigen::Vector3d center3(0.0, 1.0, 0.0);
    double area3 = 1.0;

    RoundwoodJoinery::JointFace face1(normal1, center1, area1);
    RoundwoodJoinery::JointFace face2(normal2, center2, area2);
    RoundwoodJoinery::JointFace face3(normal3, center3, area3);

    // Create a Joint instance with the created face
    std::vector<RoundwoodJoinery::JointFace> faces = {face1, face2, face3};
    RoundwoodJoinery::Joint joint(faces);

    if (joint.GetNumFaces() != 3)
    {
        std::cerr << "Test failed: Expected 3 faces, got " << joint.GetNumFaces() << std::endl;
        return 1;
    }

    if (joint.GetFaces()[0].GetArea() != 1.0)
    {
        std::cerr << "Test failed: Expected area 1.0, got " << joint.GetFaces()[0].GetArea() << std::endl;
        return 1;
    }

    return 0;
}