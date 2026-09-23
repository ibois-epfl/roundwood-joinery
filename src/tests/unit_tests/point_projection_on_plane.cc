#include "../../RoundwoodJoinery/RoundwoodJoinery.hh"
#include <iostream>

int main()
{

    Eigen::Vector3d planeNormal(0.0, 0.0, 1.0);
    Eigen::Vector3d planePoint(0.0, 0.0, 0.0);
    Eigen::Vector3d direction1(1.0, 1.0, -1.0);
    Eigen::Vector3d direction2(1.0, 0.0, 0.0);
    Eigen::Vector3d point(1.0,1.0,1.0);

    bool firstSuccess;
    bool secondSuccess;


    std::pair<Eigen::Vector3d, bool> projectedPoint1 = RoundwoodJoinery::Utils::ProjectPointOnPlaneAlongDirection(point, planePoint, planeNormal, direction1);
    firstSuccess = projectedPoint1.second;

    std::pair<Eigen::Vector3d, bool> projectedPoint2 = RoundwoodJoinery::Utils::ProjectPointOnPlaneAlongDirection(point, planePoint, planeNormal, direction2);
    secondSuccess = projectedPoint2.second;

    if (firstSuccess && !secondSuccess)
    {
        std::cout << "Test passed: Correctly handled projection along direction1 and direction2." << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Test failed: Unexpected behavior in projection." << std::endl;
        return 1;
    }

}