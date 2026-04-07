#include "Beam.hh"

namespace RoundwoodJoinery::Beam
{
    Beam::Beam(double referenceDiameter, 
        std::vector<Joinery::JointGroup> jointGroups, 
        std::vector<Eigen::Vector3d> skeleton, 
        RoundwoodJoinery::PointCloud::PointCloud pointCloud)
            : _referenceDiameter(referenceDiameter), 
              _jointGroups(jointGroups), 
              _skeleton(skeleton), 
              _pointCloud(pointCloud)
    {
        // just initialising the relative degrees of freedom matrix with zeros, it will be updated later based on the joint groups and their interactions.
        this->_relativeDegreesOfFreedom = std::vector<std::vector<Eigen::Vector3d>>(_jointGroups.size(), std::vector<Eigen::Vector3d>(_jointGroups.size(), Eigen::Vector3d::Zero()));
        for (auto& jointGroup : _jointGroups)
        {
            for (auto& joint : jointGroup.GetJoints())
            {
                joint->SetClosestPointOnSkeleton(this->_FindClosestPointOnSkeleton(joint->GetCenter()));
            }
        }
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

    std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> Beam::_ComputeJointFaceTranslationsForOptimisation()
    {
        std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> anchorPointsAndTranslations;

        for (auto& jointGroup : this->_jointGroups)
        {
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> groupTranslations;
            for (auto& joint : jointGroup.GetJoints())
            {
                for (RoundwoodJoinery::Joinery::JointFace& face : joint->GetFaces())
                {
                    Eigen::Vector3d currentCenter = face.GetCenter();
                    double targetArea = face.GetTargetArea();
                    double currentArea = face.ComputeCurrentArea(this->_pointCloud);

                    double areaRatio = currentArea / targetArea;
                    Eigen::Vector3d closestPointOnSkeleton = this->_FindClosestPointOnSkeleton(currentCenter);
                    double distanceToSkeleton = (currentCenter - closestPointOnSkeleton).norm();

                    double openingAngle = std::acos(distanceToSkeleton / (this->_referenceDiameter / 2.0));
                    double newAngle = std::asin(areaRatio * std::sin(openingAngle));

                    double translationMagnitude = distanceToSkeleton - (this->_referenceDiameter / 2.0) * std::cos(newAngle);
                    Eigen::Vector3d translationDirection = (currentCenter - closestPointOnSkeleton).normalized();
                    Eigen::Vector3d translation = translationMagnitude * translationDirection;

                    for (Eigen::Vector3d& corner : face.GetCorners())
                    {
                        groupTranslations.push_back(std::make_pair(corner, translation));
                    }
                }
            }
            anchorPointsAndTranslations.push_back(groupTranslations);
        }
        return anchorPointsAndTranslations;
    }
}