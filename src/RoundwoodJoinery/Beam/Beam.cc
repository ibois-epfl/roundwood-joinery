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
        for (Joinery::JointGroup& jointGroup : this->_jointGroups)
        {
            for (std::shared_ptr<Joinery::Joint>& joint : jointGroup.GetJoints())
            {
                Eigen::Vector3d closestPointOnSkeleton = this->_FindClosestPointOnSkeleton(joint->GetCenter());
                joint->SetClosestPointOnSkeleton(closestPointOnSkeleton);
                Eigen::Vector3d outwardDirection = (joint->GetCenter() - closestPointOnSkeleton).normalized();
                for(std::shared_ptr<Joinery::JointFace> face : joint->GetFaces())
                {
                    if (face->GetNormal().dot(outwardDirection) < 0)
                    {
                        face->FlipNormal();
                    }
                }
            }
        }
    }


     std::vector<Eigen::Matrix4d> Beam::ComputeJointGroupOptimisation(int maxIterations, double minRelativeTranslationRMSE)
     {
        // totalTransformations will accumulate the transformations applied to each joint group across iterations
        std::vector<Eigen::Matrix4d> totalTransformations(this->_jointGroups.size(), Eigen::Matrix4d::Identity());
        std::vector<Eigen::Matrix4d> previousTransformations;
        for (int iteration = 0; iteration < maxIterations; ++iteration)
        {
            std::vector<Eigen::Matrix4d> transformations = this->ComputeOneIterationOfJointFaceTranslationsForOptimisation();

            double translationRMSE = 0.0;            
            for(int i = 0; i < transformations.size(); ++i)
            {
                std::vector<Eigen::Vector3d> jointCentersBeforeTransformation;
                std::vector<Eigen::Vector3d> jointCentersAfterTransformation;

                for (auto& joint : this->_jointGroups[i].GetJoints())
                {
                    jointCentersBeforeTransformation.push_back(joint->GetCenter());
                }
                this->_jointGroups[i].ApplyTransformation(transformations[i]);

                for (auto& joint : this->_jointGroups[i].GetJoints())
                {
                    jointCentersAfterTransformation.push_back(joint->GetCenter());
                }

                for (size_t j = 0; j < jointCentersBeforeTransformation.size(); ++j)
                {
                    translationRMSE += (jointCentersAfterTransformation[j] - jointCentersBeforeTransformation[j]).squaredNorm();
                }

                totalTransformations[i] = transformations[i] * totalTransformations[i];
            }
            translationRMSE = std::sqrt(translationRMSE / this->_jointGroups.size());

            if (translationRMSE < minRelativeTranslationRMSE)
            {
                std::cout << "Convergence reached at iteration " << iteration << " with translation RMSE: " << translationRMSE << std::endl;
                return totalTransformations;
            }
            previousTransformations = transformations;
        }
     return totalTransformations;
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
                for (std::shared_ptr<RoundwoodJoinery::Joinery::JointFace>& face : joint->GetFaces())
                {
                    Eigen::Vector3d currentCenter = face->GetCenter();
                    double targetArea = face->GetTargetArea();
                    std::pair<double, double> currentAreaAndDepth = face->ComputeCurrentAreaAndDepth(this->_pointCloud);
                    double currentArea = currentAreaAndDepth.first;
                    double currentDepth = currentAreaAndDepth.second;

                    double deltaArea = (currentArea / targetArea) - 1;
                    Eigen::Vector3d closestPointOnSkeleton = this->_FindClosestPointOnSkeleton(currentCenter);

                    double translationMagnitude = deltaArea * (this->_referenceDiameter / 2.0) * 0.25; // 0.25 is a damping factor to prevent overshooting
                    double expectedNewDepth = currentDepth - translationMagnitude;

                    // Create hard floor for depth if maxProjectionDistance is set for the face
                    if(face->GetMaxProjectionDistance() > 0.0 && expectedNewDepth > face->GetMaxProjectionDistance())
                    {
                        translationMagnitude = (face->GetMaxProjectionDistance() / expectedNewDepth) * translationMagnitude;
                    }
                    Eigen::Vector3d translationDirection = face->GetNormal().normalized();
                    Eigen::Vector3d translation = translationMagnitude * translationDirection;

                    for (Eigen::Vector3d& corner : face->GetCorners())
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