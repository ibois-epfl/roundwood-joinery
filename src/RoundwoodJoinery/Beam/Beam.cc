#include "Beam.hh"

namespace RoundwoodJoinery::Beam
{
    Beam::Beam(std::vector<std::shared_ptr<Joinery::Joint>> joints, std::vector<Eigen::Vector3d> skeleton, RoundwoodJoinery::PointCloud::PointCloud pointCloud)
        : _joints(joints), _skeleton(skeleton), _pointCloud(pointCloud)
    {

    }

    Eigen::Vector3d Beam::_FindClosestPointOnSkeleton(const Eigen::Vector3d& point)
    {
        Eigen::Vector3d closestPoint = Eigen::Vector3d::Zero();
        double minDistance = std::numeric_limits<double>::max();
        int index = -1;

        for (int i = 0; i < this->_skeleton.size(); ++i)
        {
            const auto& skeletonPoint = this->_skeleton[i];
            double distance = (point - skeletonPoint).norm();
            
            if (distance < minDistance)
            {
                minDistance = distance;
                closestPoint = skeletonPoint;
                index = i;
            }
            else
            {
                // Relying on the assumption that the skeleton points are ordered, 
                // we can break early once the distance starts increasing
                break;
            }
            if (index != -1 && index < this->_skeleton.size() - 1)
            {
                Eigen::Vector3d nextSkeletonPoint = this->_skeleton[i+1];
                closestPoint = Utils::FindHeightOfTriangle(point, skeletonPoint, nextSkeletonPoint);
            }
        }
        return closestPoint;
    }
}