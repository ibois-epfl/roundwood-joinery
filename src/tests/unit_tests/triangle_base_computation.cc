#include "../../RoundwoodJoinery/RoundwoodJoinery.hh"
#include <iostream>

int main()
{
    Eigen::Vector3d testPoint(1.0, 0.0, 3.0);
    Eigen::Vector3d basePoint1(0.0, 0.0, 0.0);
    Eigen::Vector3d basePoint2(2.0, 0.0, 0.0);

    Eigen::Vector3d heightBasePoint = RoundwoodJoinery::Utils::FindHeightOfTriangle(testPoint, basePoint1, basePoint2);
    if ((heightBasePoint - Eigen::Vector3d(1.0, 0.0, 0.0)).norm() > 1e-6)
    {
        std::cerr << "Unexpected height base point: " << heightBasePoint.transpose() << std::endl;
        return 1;
    }
    return 0;
}