# include "Joint.hh"

namespace RoundwoodJoinery
{
    JointFace::JointFace(Eigen::Vector3d normal, Eigen::Vector3d center, double area)
        : normal(normal), center(center), area(area)
    {
    }

    Joint::Joint(std::vector<JointFace> faces)
        : faces(faces)
    {
    }
}