#include "../../RoundwoodJoinery/RoundwoodJoinery.hh"
#include <iostream>

int main()
{
    Eigen::Vector3d anchor1(1.0, 0.0, 0.0);
    Eigen::Vector3d anchor2(0.0, 1.0, 0.0);
    Eigen::Vector3d anchor3(0.0, 0.0, 1.0);
    Eigen::Vector3d anchor4(1.0, 1.0, 1.0);

    Eigen::Matrix4d expectedTransformation = Eigen::Matrix4d::Identity();
    expectedTransformation.block<3, 3>(0, 0) = Eigen::AngleAxisd(RoundwoodJoinery::PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Vector3d transformedAnchor1 = expectedTransformation.block<3, 3>(0, 0) * anchor1 + expectedTransformation.block<3, 1>(0, 3);
    Eigen::Vector3d transformedAnchor2 = expectedTransformation.block<3, 3>(0, 0) * anchor2 + expectedTransformation.block<3, 1>(0, 3);
    Eigen::Vector3d transformedAnchor3 = expectedTransformation.block<3, 3>(0, 0) * anchor3 + expectedTransformation.block<3, 1>(0, 3);
    Eigen::Vector3d transformedAnchor4 = expectedTransformation.block<3, 3>(0, 0) * anchor4 + expectedTransformation.block<3, 1>(0, 3);

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> anchorPointsAndTranslations = {
        {anchor1, transformedAnchor1 - anchor1},
        {anchor2, transformedAnchor2 - anchor2},
        {anchor3, transformedAnchor3 - anchor3},
        {anchor4, transformedAnchor4 - anchor4}
    };

    Eigen::Matrix4d computedTransformation = RoundwoodJoinery::Utils::ComputeApproximatingTransformation(anchorPointsAndTranslations);

    std::cout << "Expected Transformation:\n" << expectedTransformation << std::endl;
    std::cout << "Computed Transformation:\n" << computedTransformation << std::endl;

    if (computedTransformation.isApprox(expectedTransformation, 1e-6))
    {
        std::cout << "Test passed: Computed transformation is approximately equal to the expected transformation." << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Test failed: Computed transformation is not approximately equal to the expected transformation." << std::endl;
        return 1;
    }
}