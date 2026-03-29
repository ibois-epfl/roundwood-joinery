#include "Beam.hh"

namespace RoundwoodJoinery::Beam
{
    Beam::Beam(std::vector<std::shared_ptr<Joinery::Joint>> joints, std::vector<Eigen::Vector3d> skeleton, RoundwoodJoinery::PointCloud::PointCloud pointCloud)
        : _joints(joints), _skeleton(skeleton), _pointCloud(pointCloud)
    {

    }
}