#include "Beam.hh"

namespace RoundwoodJoinery::Beam
{
    Beam::Beam(double referenceDiameter, 
        std::vector<std::shared_ptr<Joinery::JointGroup>> jointGroups, 
        std::vector<Eigen::Vector3d> skeleton, 
        RoundwoodJoinery::PointCloud::PointCloud pointCloud)
            : _referenceDiameter(referenceDiameter), 
              _jointGroups(jointGroups), 
              _skeleton(skeleton), 
              _pointCloud(pointCloud)
    {
        for (std::shared_ptr<Joinery::JointGroup>& jointGroup : this->_jointGroups)
        {
            for (std::shared_ptr<Joinery::Joint>& joint : jointGroup->GetJoints())
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


    std::vector<Eigen::Matrix4d> Beam::ComputeJointGroupOptimisation(int maxIterations, double minRelativeTranslationRMSE, std::optional<std::string> outputFolderPath)
    {
        // totalTransformations will accumulate the transformations applied to each joint group across iterations
        std::vector<Eigen::Matrix4d> totalTransformations(this->_jointGroups.size(), Eigen::Matrix4d::Identity());
        std::vector<Eigen::Matrix4d> previousTransformations;

        std::ofstream outputFile;
        if (outputFolderPath.has_value())
        {
            outputFile.open(outputFolderPath.value());
            if (!outputFile.is_open())
            {
                std::cerr << "Failed to open output file for writing area ratios." << std::endl;
                return totalTransformations; // Return identity transformations if file cannot be opened
            }
            outputFile << "Iteration,JointGroupIndex,JointIndex,FaceIndex,CurrentArea,TargetArea,AreaRatio\n";
        }
        std::cout << "Starting joint group optimization with max iterations: " << maxIterations << " and minimum relative translation RMSE: " << minRelativeTranslationRMSE << std::endl;
        
        for (int iteration = 0; iteration < maxIterations; ++iteration)
        {
            std::vector<Eigen::Matrix4d> transformations = this->ComputeOneIterationOfJointFaceTranslationsForOptimisation();

            for (int i = 0; i < this->_jointGroups.size(); ++i)
            {
                std::cout << "pif" << std::endl;
                std::shared_ptr<Joinery::JointGroup>& jointGroup = this->_jointGroups[i];
                for ( int j = 0; j < jointGroup->GetJoints().size(); ++j)
                {
                    std::cout << "paf" << std::endl;
                    std::shared_ptr<Joinery::Joint>& joint = jointGroup->GetJoints()[j];
                    std::cout << "puf"<< std::endl;
                    std::cout << "joint ptr: " << joint.get() << std::endl;
                    if (joint == nullptr)
                    {
                        std::cerr << "Null joint encountered in joint group " << i << ", skipping." << std::endl;
                        continue;
                    }
                    for (size_t k = 0; k < joint->GetNumFaces(); ++k)
                    {
                        std::cout << "pouf" << std::endl;
                        std::shared_ptr<Joinery::JointFace>& face = joint->GetFaces()[k];
                        double currentArea = face->GetCurrentArea();
                        double targetArea = face->GetTargetArea();
                        double areaRatio = currentArea / targetArea;
                        if (outputFile.is_open() && outputFolderPath.has_value())
                        {
                            outputFile << iteration << "," << i << "," << j << "," << k << "," << currentArea << "," << targetArea << "," << areaRatio << "\n";
                        }
                    }
                }
            }

            double translationRMSE = 0.0;
            for (int i = 0; i < transformations.size(); ++i)
            {
                std::vector<Eigen::Vector3d> jointCentersBeforeTransformation;
                std::vector<Eigen::Vector3d> jointCentersAfterTransformation;

                for (auto& joint : this->_jointGroups[i]->GetJoints())
                {
                    jointCentersBeforeTransformation.push_back(joint->GetCenter());
                }
                this->_jointGroups[i]->ApplyTransformation(transformations[i]);

                for (auto& joint : this->_jointGroups[i]->GetJoints())
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
        if (outputFile.is_open() && outputFolderPath.has_value())
        {
            outputFile.close();
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
            for (auto& joint : jointGroup->GetJoints())
            {
                for (std::shared_ptr<RoundwoodJoinery::Joinery::JointFace>& face : joint->GetFaces())
                {
                    Eigen::Vector3d currentCenter = face->GetCenter();
                    double targetArea = face->GetTargetArea();
                    std::pair<double, double> currentAreaAndDepth = face->ComputeCurrentAreaAndDepth(this->_pointCloud, this->_referenceDiameter);
                    double currentArea = currentAreaAndDepth.first;
                    double currentDepth = currentAreaAndDepth.second;

                    double deltaArea = (currentArea / targetArea) - 1;
                    Eigen::Vector3d closestPointOnSkeleton = this->_FindClosestPointOnSkeleton(currentCenter);

                    double dampingFactor = 0.25;
                    if ((currentCenter - closestPointOnSkeleton).norm() < this->_referenceDiameter / 2.0)
                    {
                        double f = std::cos(std::asin(std::min((currentCenter - closestPointOnSkeleton).norm() / (this->_referenceDiameter / 2.0), 1.0))) + 0.01; // The +0.01 prevents the factor from becoming exactly 0, which would cause no translation and thus no optimization for faces very close to the skeleton
                        dampingFactor = f * dampingFactor;
                    }

                    double translationMagnitude = deltaArea * (this->_referenceDiameter / 2.0) * dampingFactor; // 0.25 is a damping factor to prevent overshooting
                    double expectedNewDepth = currentDepth - translationMagnitude;

                    // Create hard floor for depth if maxAllowableDepth is set for the face
                    if(face->GetMaxAllowableDepth() > 0.0 && expectedNewDepth > face->GetMaxAllowableDepth())
                    {
                        translationMagnitude = (face->GetMaxAllowableDepth() / expectedNewDepth) * translationMagnitude;
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