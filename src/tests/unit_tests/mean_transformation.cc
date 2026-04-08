#include <../../RoundwoodJoinery/RoundwoodJoinery.hh>

int main()
{
    Eigen::Matrix4d transform1 = Eigen::Matrix4d::Identity();
    transform1.block<3, 3>(0, 0) = Eigen::AngleAxisd(RoundwoodJoinery::PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    transform1.block<3, 1>(0, 3) = Eigen::Vector3d(1, 0, 0);
    Eigen::Matrix4d transform2 = Eigen::Matrix4d::Identity();
    transform2.block<3, 3>(0, 0) = Eigen::AngleAxisd(-1 * RoundwoodJoinery::PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    transform2.block<3, 1>(0, 3) = Eigen::Vector3d(-1, 0, 0);

    std::vector<Eigen::Matrix4d> transformations = {transform1, transform2};

    Eigen::Matrix4d meanTransform = RoundwoodJoinery::Utils::ComputeMeanTransformation(transformations);
    if (meanTransform.isApprox(Eigen::Matrix4d::Identity(), 1e-6))
    {
        std::cout << "Mean transformation is approximately the identity matrix, as expected." << std::endl;
    }
    else
    {
        std::cerr << "Mean transformation is not approximately the identity matrix!" << std::endl;
        std::cerr << "Mean Transformation:\n" << meanTransform << std::endl;
        return 1; // indicate failure
    }
}