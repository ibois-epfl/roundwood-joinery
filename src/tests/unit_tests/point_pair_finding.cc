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
    std::vector<Eigen::Vector3d> curve3 = {
        Eigen::Vector3d(-10.0, 0.0, 1.0),
        Eigen::Vector3d(-5.0, 0.0, 1.0),
        Eigen::Vector3d(5.0, 0.0, 1.0),
        Eigen::Vector3d(10.0, 0.0, 1.0)
    };
    std::vector<Eigen::Vector3d> curve4 = {
        Eigen::Vector3d(0.0, -10.0, -1.0),
        Eigen::Vector3d(0.0, -5.0, -1.0),
        Eigen::Vector3d(0.0, 5.0, -1.0),
        Eigen::Vector3d(0.0, 10.0, -1.0)
    };

    std::pair<Eigen::Vector3d, Eigen::Vector3d> closestPoints1 = RoundwoodJoinery::Utils::ComputeClosestPointsBetweenTwoCurves(curve1, curve2);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> closestPoints2 = RoundwoodJoinery::Utils::ComputeClosestPointsBetweenTwoCurves(curve3, curve4);

    std::cout << "Closest point on curve1: " << closestPoints1.first.transpose() << std::endl;
    std::cout << "Closest point on curve2: " << closestPoints1.second.transpose() << std::endl;
    std::cout << "Closest point on curve3: " << closestPoints2.first.transpose() << std::endl;
    std::cout << "Closest point on curve4: " << closestPoints2.second.transpose() << std::endl;

    if (closestPoints1.first.isApprox(Eigen::Vector3d(0.0, 0.0, -1.0), 1e-6) &&
        closestPoints2.first.isApprox(Eigen::Vector3d(0.0, 0.0, 1.0), 1e-6) &&
        closestPoints1.second.isApprox(Eigen::Vector3d(0.0, 0.0, 1.0), 1e-6) &&
        closestPoints2.second.isApprox(Eigen::Vector3d(0.0, 0.0, -1.0), 1e-6))
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