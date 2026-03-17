# include "Joint.hh"

namespace RoundwoodJoinery
{
    JointFace::JointFace(Eigen::Vector3d normal, Eigen::Vector3d center, double area)
        : normal(normal), center(center), _area(area)
    {
        
    }

    Joint::Joint(std::vector<JointFace> faces)
        : _faces(faces)
    {
    }

    std::vector<JointFace> Joint::GetFaces()
    {
        return this->_faces;
    }
}