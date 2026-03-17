# include "Joint.hh"

namespace RoundwoodJoinery::Joinery
{
    RoundwoodJoinery::Joinery::JointFace::JointFace(Eigen::Vector3d normal, Eigen::Vector3d center, double area)
        : normal(normal), center(center), _area(area)
    {
        
    }

    RoundwoodJoinery::Joinery::Joint::Joint(std::vector<RoundwoodJoinery::Joinery::JointFace> faces)
        : _faces(faces)
    {
    }

    std::vector<JointFace> Joint::GetFaces()
    {
        return this->_faces;
    }
}