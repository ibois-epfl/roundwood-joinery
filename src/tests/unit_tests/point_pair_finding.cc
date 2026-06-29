#include "../../RoundwoodJoinery/RoundwoodJoinery.hh"
#include <iostream>

int main()
{
    std::vector<Eigen::Vector3d> curve1 = {
        Eigen::Vector3d(-10.0, 0.0, -1.0),
        Eigen::Vector3d(10.0, 0.0, -1.0)
    };
    std::vector<Eigen::Vector3d> curve2 = {
        Eigen::Vector3d(0.0, -10.0, 1.0),
        Eigen::Vector3d(0.0, 10.0, 1.0)
    };

    std::pair<Eigen::Vector3d, Eigen::Vector3d> closestPoints = RoundwoodJoinery::Utils::ComputeClosestPointsBetweenTwoCurves(curve1, curve2);

    std::cout << "Closest point on curve1: " << closestPoints.first.transpose() << std::endl;
    std::cout << "Closest point on curve2: " << closestPoints.second.transpose() << std::endl;

    if (closestPoints.first.isApprox(Eigen::Vector3d(0.0, 0.0, -1.0), 1e-6) &&
        closestPoints.second.isApprox(Eigen::Vector3d(0.0, 0.0, 1.0), 1e-6))
    {
        std::cout << "Test passed: Closest points are as expected." << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Test failed: Closest points are not as expected." << std::endl;
        return 1;
    }
}